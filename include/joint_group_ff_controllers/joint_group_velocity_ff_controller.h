#pragma once

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <joint_group_ff_controllers/setpoint.h>
#include <joint_group_ff_controllers/rt_stopwatch.h>

namespace joint_group_ff_controllers
{

/**
 * \brief Tracks position, velocity commands for a set of joints using a proportionnal+feedforward controller.
 *
 * The controller has two functionning mode :
 * - The normal one that applies a feedforward-P,
 * - The safety one that tries to keep the joint in their current position, using a simple P, if the timeout is reached (for safety purposes).
 *
 * \section ROS interface
 *
 * \param type Must be "joint_group_ff_controllers/JointGroupVelocityFFController".
 * \param joints List of names of the joints to control.
 * For each joint :
 * \param <joint>/kp Position gain
 * \param <joint>/kp_safe Position gain when timeout occured
 *
 * Subscribes to:
 * - \b command (joint_group_ff_controllers::setpoint) : The joint positions and velocities to apply (efforts filed is ignored), aswell as the timeout value. (if timeout < 0, then no timeout will be applied)
 */
class JointGroupVelocityFFController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  JointGroupVelocityFFController();
  ~JointGroupVelocityFFController();

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<joint_group_ff_controllers::setpoint> commands_buffer_;
  unsigned int n_joints_;

  // Gains
  std::vector<double> kp_;
  std::vector<double> kp_safe_;

  // Command subscriber
  ros::Subscriber sub_command_;
  void commandCB(const joint_group_ff_controllers::setpointConstPtr& msg);

  // Timeout purposes
  bool are_positions_held_; /* Has the timeout already been reached */
  std::vector<double> held_positions_; /* Last joint position before the tiemout has been reached */
  RTStopwatch last_command_; /* Keep the time since the last command has been received */
}; // class

} // namespace
