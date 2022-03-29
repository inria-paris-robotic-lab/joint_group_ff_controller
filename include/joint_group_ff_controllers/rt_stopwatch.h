#ifndef JOINT_GROUP_FF_CONTROLLERS__RT_STOPWATCH__
#define JOINT_GROUP_FF_CONTROLLERS__RT_STOPWATCH__

#include <mutex>
#include <ros/duration.h>

namespace joint_group_ff_controllers {

/**
 * @brief Keep track of real-time safe counter (of type ros::Duration). The counter can be incremented and read, or, reset.
 */
class RTStopwatch
{
public:
  /**
   * @brief Construct a new RTStopwatch object and reset the counter.
   */
  RTStopwatch() : counter_(0)
  {}

  /**
   * @brief Add some duration to the counter and return it's new value.
   *
   * @param value The duration to add.
   * @return ros::Duration The new value of the counter.
   */
  ros::Duration incr(const ros::Duration& value)
  {
    std::lock_guard<std::mutex> guard(counter_lock_RT_);
    counter_ += value;
    return counter_;
  }

  /**
   * @brief Reset the counter to zero.
   */
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