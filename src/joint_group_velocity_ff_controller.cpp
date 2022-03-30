#include <joint_group_ff_controllers/joint_group_velocity_ff_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace joint_group_ff_controllers
{

  JointGroupVelocityFFController::JointGroupVelocityFFController() {}
  JointGroupVelocityFFController::~JointGroupVelocityFFController() {sub_command_.shutdown();}

  bool JointGroupVelocityFFController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Init gains
    kp_.resize(n_joints_);
    kp_safe_.resize(n_joints_);

    // Init joints and parameters
    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      // Fetch gains
      if(!n.getParam(joint_name + "/kp", kp_[i]) ||
         !n.getParam(joint_name + "/kp_safe", kp_safe_[i])
         )
      {
        ROS_ERROR_STREAM("Unable to read kp, kp_safe parameters for joint: " << joint_name);
        return false;
      }
    }

    // Init commands
    joint_group_ff_controllers::setpoint dummy;
    dummy.timeout = ros::Duration(0);
    commands_buffer_.writeFromNonRT(dummy);

    // Init safety measures
    are_positions_held_ = false;
    held_positions_ = std::vector<double>(n_joints_);

    sub_command_ = n.subscribe<joint_group_ff_controllers::setpoint>("command", 1, &JointGroupVelocityFFController::commandCB, this);
    return true;
  }

  void JointGroupVelocityFFController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Get the latest command
    joint_group_ff_controllers::setpoint& commands_ = *commands_buffer_.readFromRT();

    // The timeout is reached if the last command as been received for longer that the timeout.
    // To simplify some problems, the timeout is instantaneously reached if the timeout value is zero.
    // The timeout is not checked it the value is negative.
    bool timeout = commands_.timeout.isZero() || (commands_.timeout > ros::Duration(0) && last_command_.incr(period) > commands_.timeout);
    for(unsigned int i=0; i<n_joints_; i++)
    {
        if(!are_positions_held_) { //first time that the timeout reached => save current robot position
          held_positions_[i] = joints_[i].getPosition();
        }

        // Command for the joint, depending on the mode
        double command_position = timeout ? held_positions_[i] : commands_.positions [i];
        double command_velocity = timeout ? 0                  : commands_.velocities[i];

        // Read joint
        double current_position = joints_[i].getPosition();

        // Compute position error
        double position_error = command_position - current_position;

        // Compute control
        double control_velocity = (timeout ? kp_safe_[i] : kp_[i]) * position_error
                                + command_velocity;

        joints_[i].setCommand(control_velocity);
    }
    // If the timeout was reached in the loop, print a user warning
    if(timeout && !are_positions_held_) {
      ROS_WARN_STREAM("Timeout reached, controller now try to hold current position until next valid command");
    }
    // Update tiemout status
    are_positions_held_ = timeout;
  }

  void JointGroupVelocityFFController::commandCB(const joint_group_ff_controllers::setpointConstPtr& msg)
  {
    // Filter message based on commands sizes
    if(msg->positions.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command positions (" << msg->positions.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    if(msg->velocities.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command velocities (" << msg->velocities.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    // Save latest commands
    commands_buffer_.writeFromNonRT(*msg);

    // Reset timeout stopwatch
    last_command_.reset();
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( joint_group_ff_controllers::JointGroupVelocityFFController, controller_interface::ControllerBase)
