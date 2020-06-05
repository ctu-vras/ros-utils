#include <cras_cpp_common/nodelet_utils.hpp>

#include <pthread.h>

namespace cras
{

static const double CAN_TRANSFORM_POLLING_SCALE = 0.01;

Nodelet::~Nodelet() {
  this->shutdown();
}

void Nodelet::updateThreadName() const
{
  char nameBuf[16];
  const auto& name = stripLeadingSlash(::nodelet::Nodelet::getName());

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

bool Nodelet::ok() const {
  return !this->isUnloading;
}

void Nodelet::shutdown() {
  this->isUnloading = true;
}

Nodelet::Nodelet() : ::nodelet::Nodelet(), NodeletParamHelper(::nodelet::Nodelet::getName()) {
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

NodeletAwareTFBuffer::NodeletAwareTFBuffer(const Nodelet* nodelet) :
  nodelet(nodelet)
{
}

}