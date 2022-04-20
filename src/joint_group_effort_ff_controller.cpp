#include <joint_group_ff_controllers/joint_group_effort_ff_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace joint_group_ff_controllers
{

  JointGroupEffortFFController::JointGroupEffortFFController() {}
  JointGroupEffortFFController::~JointGroupEffortFFController() {sub_command_.shutdown();}

  bool JointGroupEffortFFController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
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
    kd_.resize(n_joints_);
    kp_safe_.resize(n_joints_);
    kd_safe_.resize(n_joints_);
    hrdw_cst_.resize(n_joints_);

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
         !n.getParam(joint_name + "/kd", kd_[i]) ||
         !n.getParam(joint_name + "/kp_safe", kp_safe_[i]) ||
         !n.getParam(joint_name + "/kd_safe", kd_safe_[i])
         )
      {
        ROS_ERROR_STREAM("Unable to read kp, kd, kp_safe, kd_safe parameters for joint: " << joint_name);
        return false;
      }

      // Fetch optionnal parameters
      if(n.hasParam(joint_name + "/hardware")) {
        double mot_cst_;
        double red_cst_;
        if(!n.getParam(joint_name + "/hardware/motor_torque_constant", mot_cst_) ||
           !n.getParam(joint_name + "/hardware/reduction_ratio", red_cst_)) {
             ROS_ERROR_STREAM("Hardware constant badly defined. Both " << joint_name << "/hardware/motor_torque_constant and " << joint_name << "/hardware/reduction_ratio should be either defined or undefined.");
             return false;
           }
          hrdw_cst_[i] = 1 / mot_cst_ * red_cst_;
      } else {
        hrdw_cst_[i] = 1;
      }
    }

    // Init commands
    joint_group_ff_controllers::setpoint dummy;
    dummy.timeout = ros::Duration(0);
    commands_buffer_.writeFromNonRT(dummy);

    // Init safety measures
    are_positions_held_ = false;
    held_positions_ = std::vector<double>(n_joints_);

    sub_command_ = n.subscribe<joint_group_ff_controllers::setpoint>("command", 1, &JointGroupEffortFFController::commandCB, this);
    return true;
  }

  void JointGroupEffortFFController::update(const ros::Time& time, const ros::Duration& period)
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
        double command_effort   = timeout ? 0                  : commands_.efforts   [i];

        // Read joint
        double current_position = joints_[i].getPosition();
        double current_velocity = joints_[i].getVelocity();

        // Compute position error
        double position_error = command_position - current_position;
        double velocity_error = command_velocity - current_velocity;

        // Compute control
        double control_effort = (timeout ? kp_safe_[i] : kp_[i]) * position_error
                                + (timeout ? kd_safe_[i] : kd_[i]) * velocity_error
                                + command_effort;

        // Apply hardware constant
        control_effort *= hrdw_cst_[i];

        joints_[i].setCommand(control_effort);
    }
    // If the timeout was reached in the loop, print a user warning
    if(timeout && !are_positions_held_) {
      ROS_WARN_STREAM("Timeout reached, controller now try to hold current position until next valid command");
    }
    // Update tiemout status
    are_positions_held_ = timeout;
  }

  void JointGroupEffortFFController::commandCB(const joint_group_ff_controllers::setpointConstPtr& msg)
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
    if(msg->efforts.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command efforts (" << msg->efforts.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    // Save latest commands
    commands_buffer_.writeFromNonRT(*msg);

    // Reset timeout stopwatch
    last_command_.reset();
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( joint_group_ff_controllers::JointGroupEffortFFController, controller_interface::ControllerBase)
