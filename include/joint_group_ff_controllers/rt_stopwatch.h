#ifndef JOINT_GROUP_FF_CONTROLLERS__RT_STOPWATCH__
#define JOINT_GROUP_FF_CONTROLLERS__RT_STOPWATCH__

#include <mutex>
#include <ros/duration.h>

namespace joint_group_ff_controllers {

class RTStopwatch
{
public:
  RTStopwatch() : counter_(0)
  {
  }

  ros::Duration incr(const ros::Duration& value)
  {
    std::lock_guard<std::mutex> guard(counter_lock_RT_);
    counter_ += value;
    return counter_;
  }

  void reset()
  {
    std::lock_guard<std::mutex> guard(counter_lock_RT_);
    counter_ = ros::Duration(0);
  }

private:
  ros::Duration counter_;

  std::mutex counter_lock_RT_;
};

} // namespace

#endif //JOINT_GROUP_FF_CONTROLLERS__RT_STOPWATCH__