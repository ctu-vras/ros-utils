// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Priority-based muxer for topics.
 * \author Martin Pecka
 */

#include <string>
#include <unordered_map>
#include <utility>

#include <cras_cpp_common/log_utils.h>

#include <cras_topic_tools/priority_mux_base.h>

namespace cras
{

PriorityMux::PriorityMux(const std::unordered_map<std::string, cras::priority_mux::TopicConfig>& topicConfigs,
                         const std::unordered_map<std::string, cras::priority_mux::LockConfig>& lockConfigs,
                         const cras::PriorityMux::SetTimerFn& setTimerFn, const ::ros::Time& now,
                         const cras::LogHelperPtr& log, const std::string& noneTopic, int nonePriority) :
  cras::HasLogger(log), topicConfigs(topicConfigs), lockConfigs(lockConfigs), setTimer(setTimerFn),
  noneTopic(noneTopic), nonePriority(nonePriority)
{
  this->resetImpl(now);

  CRAS_INFO("Starting with priority %i.", nonePriority);
  for (const auto& outAndSelectedTopic : this->lastSelectedTopics)
    CRAS_DEBUG("Starting with no selected topic for output topic %s.", outAndSelectedTopic.first.c_str());

  for (const auto& nameAndLockConfig : this->lockConfigs)
  {
    const auto& name = nameAndLockConfig.first;
    const auto& config = nameAndLockConfig.second;
    if (config.timeout != ros::Duration(0, 0))
      this->setTimer(name, config.timeout);
  }
}

bool PriorityMux::cb(const std::string& inTopic, const ros::Time& stamp, const ros::Time& now)
{
  const auto it = this->topicConfigs.find(inTopic);
  if (it == this->topicConfigs.end())
  {
    CRAS_ERROR("Priority mux called with topic %s which is not configured.", inTopic.c_str());
    return false;
  }

  const auto& topicConfig = it->second;
  const auto& priority = topicConfig.priority;
  this->lastReceiveStamps[std::make_pair(priority, inTopic)] = stamp;

  this->update(now);

  // If stamp is older than now, wait only for as much time as actually remains
  const auto remainingTimeout = topicConfig.timeout + (stamp - now);
  // If the callback came late and the topic is already timed out, do not plan a callback as everything is done in the
  // preceding update() call.
  if (remainingTimeout > ros::Duration(0, 0))
    this->setTimer(inTopic, remainingTimeout);

  return this->lastActivePriority <= priority;
}

void PriorityMux::disableCb(const std::string& inTopic, const ros::Time& stamp, const bool disable,
                            const ros::Time& now)
{
  const auto it = this->topicConfigs.find(inTopic);
  if (it == this->topicConfigs.end())
  {
    CRAS_ERROR("Priority mux called with topic %s which is not configured.", inTopic.c_str());
    return;
  }

  const auto& topicConfig = it->second;
  const auto& priority = topicConfig.priority;
  const auto key = std::make_pair(priority, inTopic);

  if (disable)
    this->disabledStamps[key] = stamp;
  else
    this->disabledStamps.erase(key);

  this->update(now);

  if (disable)
  {
    // If stamp is older than now, wait only for as much time as actually remains
    const auto remainingTimeout = topicConfig.timeout + (stamp - now);
    // If the callback came late and the topic is already timed out, do not plan a callback as everything is done in the
    // preceding update() call.
    if (remainingTimeout > ros::Duration(0, 0))
      this->setTimer("__disable_" + inTopic, remainingTimeout);
  }
}

void PriorityMux::lockCb(const std::string& topic, const ros::Time& stamp, const bool locked, const ros::Time& now)
{
  const auto it = this->lockConfigs.find(topic);
  if (it == this->lockConfigs.end())
  {
    CRAS_ERROR("Priority mux called with lock topic %s which is not configured.", topic.c_str());
    return;
  }

  const auto& lockConfig = it->second;
  const auto& priority = lockConfig.priority;
  const auto& timeout = lockConfig.timeout;

  const auto key = std::make_pair(priority, topic);
  this->lastLockStamps[key] = stamp;
  this->lockStates[key] = locked;

  this->update(now);

  // If stamp is older than now, wait only for as much time as actually remains
  const auto remainingTimeout = timeout + (stamp - now);

  if (!locked && timeout != ros::Duration(0, 0) && remainingTimeout > ros::Duration(0, 0))
    this->setTimer(topic, timeout);
}

void PriorityMux::update(const ros::Time& now)
{
  const auto lockedPriority = this->getHighestLockedPriority(now);

  // Iterate from the highest priority downwards and find the first non-timeouted non-disabled topic
  int topicsPriority {this->nonePriority};
  for (auto it = this->lastReceiveStamps.crbegin(); it != this->lastReceiveStamps.crend(); ++it)
  {
    const auto& itPriority = it->first.first;
    const auto& itTopic = it->first.second;
    const auto& itConfig = this->topicConfigs[itTopic];
    const auto& itStamp = it->second;
    if (now < itStamp + itConfig.timeout)
    {
      // Check if the topic isn't currently disabled
      const auto disableIt = this->disabledStamps.find(it->first);
      if (disableIt != this->disabledStamps.end())
      {
        const auto& disabledStamp = disableIt->second;
        if (now < disabledStamp + itConfig.timeout)  // The topic is disabled, skip it
          continue;
        // If the disable is timed out, delete its record.
        this->disabledStamps.erase(it->first);
      }
      topicsPriority = itPriority;
      break;
    }
  }

  const auto lastPriority = this->lastActivePriority;
  this->lastActivePriority = (std::max)(topicsPriority, lockedPriority);

  if (lastPriority != this->lastActivePriority)
    CRAS_INFO("Changed priority from %i to %i.", lastPriority, this->lastActivePriority);

  auto lastTopics = this->lastSelectedTopics;

  for (auto& selected : this->lastSelectedTopics)
    selected.second = this->noneTopic;

  if (topicsPriority >= lockedPriority)
  {
    for (const auto& topicAndConfig : this->topicConfigs)
    {
      const auto& config = topicAndConfig.second;
      if (config.priority != topicsPriority)
        continue;
      this->lastSelectedTopics[config.outTopic] = topicAndConfig.first;
    }
  }

  for (const auto& outAndSelectedTopic : this->lastSelectedTopics)
  {
    if (lastTopics[outAndSelectedTopic.first] != outAndSelectedTopic.second)
      CRAS_DEBUG("Choosing topic %s instead of %s for output topic %s.", outAndSelectedTopic.second.c_str(),
                 lastTopics[outAndSelectedTopic.first].c_str(), outAndSelectedTopic.first.c_str());
  }
}

int PriorityMux::getHighestLockedPriority(const ros::Time& now)
{
  auto highestLockedPriority = this->nonePriority;
  for (auto it = this->lastLockStamps.crbegin(); it != this->lastLockStamps.crend(); ++it)
  {
    const auto& itPriority = it->first.first;
    const auto& itTopic = it->first.second;
    const auto& itStamp = it->second;
    const auto& itConfig = this->lockConfigs[itTopic];
    const auto& itTimeout = itConfig.timeout;
    const auto key = std::make_pair(itPriority, itTopic);
    if ((itTimeout != ros::Duration(0, 0) && itStamp + itTimeout < now) || this->lockStates[key])
    {
      highestLockedPriority = itPriority;
      break;
    }
  }
  return highestLockedPriority;
}

const std::unordered_map<std::string, std::string>& PriorityMux::getLastSelectedTopics() const
{
  return this->lastSelectedTopics;
}

int PriorityMux::getActivePriority() const
{
  return this->lastActivePriority;
}

void PriorityMux::reset(const ::ros::Time& now)
{
  this->resetImpl(now);
}

void PriorityMux::resetImpl(const ::ros::Time& now)
{
  this->lastActivePriority = this->nonePriority;
  this->lastReceiveStamps.clear();

  for (const auto& topicAndConfig : this->topicConfigs)
    this->lastSelectedTopics[topicAndConfig.second.outTopic] = this->noneTopic;

  for (const auto& topicAndLockConfig : this->lockConfigs)
  {
    const auto key = std::make_pair(topicAndLockConfig.second.priority, topicAndLockConfig.first);
    this->lastLockStamps[key] = now;
    this->lockStates[key] = false;
  }

  this->disabledStamps.clear();

  this->update(now);
}

}
