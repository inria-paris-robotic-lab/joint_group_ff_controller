#pragma once

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <joint_group_ff_controllers/effort_command.h>
#include <joint_group_ff_controllers/rt_stopwatch.h>

namespace joint_group_ff_controllers
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
class JointGroupEffortFFController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  JointGroupEffortFFController();
  ~JointGroupEffortFFController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<joint_group_ff_controllers::effort_command> commands_buffer_;
  unsigned int n_joints_;

  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> kp_safe_;
  std::vector<double> kd_safe_;

  ros::Subscriber sub_command_;
  void commandCB(const joint_group_ff_controllers::effort_commandConstPtr& msg);

  bool are_positions_held_;
  std::vector<double> held_positions_;
  RTStopwatch last_command_;
}; // class

} // namespace
