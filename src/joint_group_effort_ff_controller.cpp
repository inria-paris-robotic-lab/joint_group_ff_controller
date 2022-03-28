/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <joint_group_ff_controllers/joint_group_effort_ff_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

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
    }

    // Init commands
    joint_group_ff_controllers::effort_command dummy;
    dummy.positions = std::vector<double>(n_joints_, 0.);
    dummy.velocities = std::vector<double>(n_joints_, 0.);
    dummy.efforts = std::vector<double>(n_joints_, 0.);
    dummy.timeout = ros::Duration(0);
    commands_buffer_.writeFromNonRT(dummy);

    sub_command_ = n.subscribe<joint_group_ff_controllers::effort_command>("command", 1, &JointGroupEffortFFController::commandCB, this);
    return true;
  }

  void JointGroupEffortFFController::update(const ros::Time& time, const ros::Duration& period)
  {
    joint_group_ff_controllers::effort_command& commands_ = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands_.positions [i];
        double command_velocity = commands_.velocities[i];
        double command_effort   = commands_.efforts   [i];

        double current_position = joints_[i].getPosition();
        double current_velocity = joints_[i].getVelocity();

        // Compute position error
        double position_error = command_position - current_position;
        double velocity_error = command_velocity - current_velocity;

        // Compute command
        double commanded_effort = kp_[i] * position_error + kd_[i] * velocity_error + command_effort;

        joints_[i].setCommand(commanded_effort);
    }
  }

  void JointGroupEffortFFController::commandCB(const joint_group_ff_controllers::effort_commandConstPtr& msg)
  {
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
    commands_buffer_.writeFromNonRT(*msg);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( joint_group_ff_controllers::JointGroupEffortFFController, controller_interface::ControllerBase)
