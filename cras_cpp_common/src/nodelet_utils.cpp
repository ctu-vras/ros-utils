#include <sstream>
#define protected public
#include <nodelet/nodelet.h>
#undef protected

#include <cras_cpp_common/nodelet_utils.hpp>

#include <pthread.h>
#include <unistd.h>

namespace cras
{

static const double CAN_TRANSFORM_POLLING_SCALE = 0.01;

StatefulNodelet::~StatefulNodelet() {
  this->shutdown();
}

void ThreadNameUpdatingNodelet::updateThreadName() const
{
  const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
  if (nodelet == nullptr)
    return;

  char nameBuf[16];
  const auto& name = stripLeadingSlash(nodelet->getName());

  if (name.length() <= 15) {
    memcpy(nameBuf, name.c_str(), name.length());
    nameBuf[name.length()] = '\0';
  } else {
    memcpy(nameBuf, name.c_str(), 7);
    memset(nameBuf + 7, '.', 1);
    memcpy(nameBuf + 8, name.c_str() + (name.length() - 7), 7);
    nameBuf[15] = '\0';
  }
  pthread_setname_np(pthread_self(), nameBuf);
}

/** This is a workaround for the case that we're running inside of
    rospy and ros::Time is not initialized inside the c++ instance.
    This makes the system fall back to Wall time if not initialized.
*/
ros::Time now_fallback_to_wall()
{
  try
  {
    return ros::Time::now();
  }
  catch (const ros::TimeNotInitializedException& ex)
  {
    ros::WallTime wt = ros::WallTime::now();
    return {wt.sec, wt.nsec};
  }
}

/** This is a workaround for the case that we're running inside of
    rospy and ros::Time is not initialized inside the c++ instance.
    This makes the system fall back to Wall time if not initialized.
    https://github.com/ros/geometry/issues/30
*/
void sleep_fallback_to_wall(const ros::Duration& d)
{
  try
  {
    d.sleep();
  }
  catch (const ros::TimeNotInitializedException& ex)
  {
    ros::WallDuration wd = ros::WallDuration(d.sec, d.nsec);
    wd.sleep();
  }
}

void conditionally_append_timeout_info(std::string * errstr, const ros::Time& start_time,
                                       const ros::Duration& timeout)
{
  if (errstr)
  {
    std::stringstream ss;
    ss << ". canTransform returned after "<< (now_fallback_to_wall() - start_time).toSec() \
       <<" timeout was " << timeout.toSec() << ".";
    (*errstr) += ss.str();
  }
}

bool StatefulNodelet::ok() const {
  return !this->isUnloading;
}

void StatefulNodelet::shutdown() {
  this->isUnloading = true;
}

bool NodeletAwareTFBuffer::canTransform(const std::string& target_frame,
    const std::string& source_frame, const ros::Time& time,
    const ros::Duration timeout, std::string *errstr) const {
  // Clear the errstr before populating it if it's valid.
  if (errstr)
  {
    errstr->clear();
  }

  // poll for transform if timeout is set
  ros::Time start_time = now_fallback_to_wall();
  const ros::Duration sleep_duration = timeout * CAN_TRANSFORM_POLLING_SCALE;
  while (now_fallback_to_wall() < start_time + timeout &&
      !((tf2::BufferCore*)this)->canTransform(target_frame, source_frame, time) &&
      (now_fallback_to_wall()+ros::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
      (ros::ok() || !ros::isInitialized()) && // Make sure we haven't been stopped (won't work for pytf)
      nodelet->ok()) // Make sure the nodelet is not requested to unload
  {
    sleep_fallback_to_wall(sleep_duration);
  }
  bool retval = ((tf2::BufferCore*)this)->canTransform(target_frame, source_frame, time, errstr);
  conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}
bool NodeletAwareTFBuffer::canTransform(const std::string &target_frame,
    const ros::Time &target_time, const std::string &source_frame,
    const ros::Time &source_time, const std::string &fixed_frame,
    const ros::Duration timeout, std::string *errstr) const {
  // Clear the errstr before populating it if it's valid.
  if (errstr)
  {
    errstr->clear();
  }

  // poll for transform if timeout is set
  ros::Time start_time = now_fallback_to_wall();
  const ros::Duration sleep_duration = timeout * CAN_TRANSFORM_POLLING_SCALE;
  while (now_fallback_to_wall() < start_time + timeout &&
      !((tf2::BufferCore*)this)->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame) &&
      (now_fallback_to_wall()+ros::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
      (ros::ok() || !ros::isInitialized()) && // Make sure we haven't been stopped (won't work for pytf)
      nodelet->ok())
  {
    sleep_fallback_to_wall(sleep_duration);
  }
  bool retval = ((tf2::BufferCore*)this)->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
  conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}

NodeletAwareTFBuffer::NodeletAwareTFBuffer(const StatefulNodelet* nodelet) :
  nodelet(nodelet)
{
}

const std::string &NodeletParamHelper::getName() const {
  const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
  if (nodelet != nullptr)
    return nodelet->getName();
  return this->defaultName;
}

struct NodeletWithSharedTfBuffer::NodeletWithSharedTfBufferPrivate
{
  std::string defaultName = "NodeletWithSharedTfBuffer_has_to_be_a_sister_class_of_Nodelet";
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  bool usesSharedBuffer = false;
};

NodeletWithSharedTfBuffer::NodeletWithSharedTfBuffer()
    : data(new NodeletWithSharedTfBufferPrivate)
{
}

void NodeletWithSharedTfBuffer::setBuffer(const std::shared_ptr<tf2_ros::Buffer> &buffer) {
  if (this->data->buffer != nullptr || this->data->listener != nullptr)
    throw std::runtime_error("tf2 buffer cannot be set multiple times");

  NODELET_INFO("Initialized shared tf2 buffer");
  this->data->buffer = buffer;
  this->data->usesSharedBuffer = true;
}

tf2_ros::Buffer& NodeletWithSharedTfBuffer::getBuffer() const {
  if (this->data->buffer == nullptr)
  {
    this->data->buffer.reset(new tf2_ros::Buffer);
    this->data->listener.reset(new tf2_ros::TransformListener(*this->data->buffer));
    this->data->usesSharedBuffer = false;
    NODELET_INFO("Initialized standalone tf2 buffer");
  }
  return *this->data->buffer;
}

const std::string& NodeletWithSharedTfBuffer::getName() const {
  const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
  if (nodelet != nullptr)
    return nodelet->getName();
  return this->data->defaultName;
}

bool NodeletWithSharedTfBuffer::usesSharedBuffer() const {
  return this->data->usesSharedBuffer;
}

NodeletWithSharedTfBuffer::~NodeletWithSharedTfBuffer() {
}

struct NodeletWithDiagnostics::NodeletWithDiagnosticsPrivate
{
  std::shared_ptr<diagnostic_updater::Updater> updater;
};

NodeletWithDiagnostics::NodeletWithDiagnostics()
    : data(new NodeletWithDiagnosticsPrivate)
{
}

diagnostic_updater::Updater& NodeletWithDiagnostics::getDiagUpdater() const {
  if (this->data->updater == nullptr)
  {
    const auto* nodelet = dynamic_cast<const ::nodelet::Nodelet*>(this);
    if (nodelet != nullptr) {
      this->data->updater = std::make_shared<diagnostic_updater::Updater>(
          nodelet->getNodeHandle(), nodelet->getPrivateNodeHandle(), nodelet->getName());
    } else {
      this->data->updater = std::make_shared<diagnostic_updater::Updater>();
    }
    char hostname[1024];
    hostname[1023] = '\0';
    gethostname(hostname, 1023);
    this->data->updater->setHardwareID(std::string(hostname));
  }
  return *this->data->updater;
}

NodeletWithDiagnostics::~NodeletWithDiagnostics() {
}

}