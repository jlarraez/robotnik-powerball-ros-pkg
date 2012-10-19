/* powerball_cart_controller.cpp
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief Implements the cartesian/euler control in KDL for the Powerball arm.
 */

#include "robotnik_powerball_sim/powerball_cart_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace powerball_cart_controller_ns;


/// Controller initialization in non-realtime                                                                                                                                     
bool PowerballCartController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
	
  // gets root and tip name from parameter server
  std::string root_name, tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // constructs a chain from root to tip
  if (!chain_.init(robot, root_name, tip_name))
  {
    ROS_ERROR("PowerballCartController could not find a chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
	    return false;
  }

  // constructs the kdl solvers in non-realtime
  KDL::Chain kdl_chain;
  chain_.toKDL(kdl_chain);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
  jnt_to_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain, 0.00001, 150));

  // resizes the joint state vectors in non-realtime
  jnt_pos_.resize(kdl_chain.getNrOfJoints());
  jnt_ref_vel_.resize(kdl_chain.getNrOfJoints());
  jnt_real_vel_.resize(kdl_chain.getNrOfJoints());
  ev_.resize(kdl_chain.getNrOfJoints());
  jnt_ref_pos_.resize(kdl_chain.getNrOfJoints());
                               
  // Listen through the node handle to the arm references provided by the robotnik_powerball_pad
  arm_ref_sub_ = n.subscribe<robotnik_powerball_pad::ArmRef>("/robotnik_powerball_pad/arm_reference", 10, &PowerballCartController::refCallback, this);
  
  // Publish the references calculated to the controller
  arm_com_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
  
  chain_.getPositions(jnt_pos_);
  
  pt_ant.positions.resize(6);
  
  for(int i = 0; i < 6; i++){
	pt_ant.positions[i] = jnt_pos_(i);
  }
  
  joint_names.push_back("arm_1_joint"); 
  joint_names.push_back("arm_2_joint"); 
  joint_names.push_back("arm_3_joint"); 
  joint_names.push_back("arm_4_joint"); 
  joint_names.push_back("arm_5_joint"); 
  joint_names.push_back("arm_6_joint"); 

  check_self_collision = true; 
  check_joint_limits = true;
  no_self_collision = false;
  n.param("check_self_collision", check_self_collision, true);
  n.param("check_joint_limits", check_joint_limits, true);
  
  // Define joint limits
  
  joint_limits[0] = 2.97;
  joint_limits[1] = 1.92;
  joint_limits[2] = 2.71;
  joint_limits[3] = 2.97;
  joint_limits[4] = 2.44;
  joint_limits[5] = 2.97;
  

  return true;
}


/// Controller startup in realtime
void PowerballCartController::starting()
{}

/// Controller update loop in realtime
void PowerballCartController::update()
{
	
  // get the joint positions of the chain
  chain_.getPositions(jnt_pos_);

  // computes Cartesian pose in realtime (not used in computation, only for send via topic)
  KDL::Frame current_pose;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, current_pose);

  // computes the velocity necessary to move to the dessired position
  jnt_to_vel_solver_->CartToJnt(jnt_pos_, incr_pos_ , jnt_ref_vel_);
  
  // get the joint velocities of the chain
  chain_.getVelocities(jnt_real_vel_);
  
  for (int i = 0; i < 6; i++){
		jnt_ref_pos_(i) =  jnt_pos_(i) + jnt_ref_vel_(i);
  }
  
  // 18/10/2012 limit articulations
  for (int i = 0; i < 6; i++){
		if(jnt_ref_pos_(i) > joint_limits[i]){
			jnt_ref_pos_(i) =  joint_limits[i];
		}else if ( jnt_ref_pos_(i) < ( joint_limits[i] * -1) ){
			jnt_ref_pos_(i) = joint_limits[i] * -1;
		}
  }
 
  
  trajectory_msgs::JointTrajectory jt;
  trajectory_msgs::JointTrajectoryPoint pt;
   
  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "/base_link";

  jt.joint_names.push_back("arm_1_joint");
  jt.joint_names.push_back("arm_2_joint");
  jt.joint_names.push_back("arm_3_joint");
  jt.joint_names.push_back("arm_4_joint");
  jt.joint_names.push_back("arm_5_joint");
  jt.joint_names.push_back("arm_6_joint");

  pt.positions.resize(6);
  
  bool changeConf = false;

  for (int i = 0; i < 6; i++) {
	  
 	pt.positions[i] = jnt_ref_pos_(i);
	
 	if ( (pt.positions[i] - pt_ant.positions[i] > 10) || (pt.positions[i] - pt_ant.positions[i] < -10) ){
		changeConf = true;
	}

  }
   
  if (changeConf){
 	//ROS_WARN("CAUTION, CHANGE OF CONFIGURATION!");
	for(int i = 0; i < 6; i++){
		pt.positions[i] = pt.positions[i]/8;
	}
  }

  pt.time_from_start = ros::Duration(20);

  jt.points.push_back(pt);

  pt_ant = pt;

  arm_com_pub_.publish(jt);	
  	
}


/// Controller stopping in realtime
void PowerballCartController::stopping()
{}

/// Get joystick increments
void PowerballCartController::refCallback(const robotnik_powerball_pad::ArmRef::ConstPtr& msg)
{
  
  // Manage the current operation mode
  if (msg->aux_reference == true){
	  if (opMode == CARTESIAN_MODE){
		  opMode = EULER_MODE;
		  ROS_ERROR("Changed to EULER mode.");
	  }else if (opMode == EULER_MODE){
		  opMode = CARTESIAN_MODE;
		  ROS_ERROR("Changed to CARTESIAN mode.");
	  }
  }
	
  if (opMode == CARTESIAN_MODE){ 	// Cartesian Mode
  
    prx_ = msg->x_position_reference * 0.2;
    pry_ = msg->y_position_reference * 0.2;
    prz_ = msg->z_position_reference * 0.2;
	  
	KDL::Vector vel(prx_, pry_, prz_);
	KDL::Vector rot(0,0,0);
	incr_pos_ = KDL::Twist(vel, rot);
	
  }else if (opMode == EULER_MODE){	// Euler mode
  
    prx_ = msg->x_position_reference * 0.5;
    pry_ = msg->y_position_reference * 0.5;
    prz_ = msg->z_position_reference * 0.5;
	  
	KDL::Vector vel(0, 0, 0);
	KDL::Vector rot(prx_, pry_, prz_);
	incr_pos_ = KDL::Twist(vel, rot);
	
  }
  
}

/// Register controller to pluginlib                                                                                                                                              
PLUGINLIB_REGISTER_CLASS(PowerballCartControllerPlugin,
                         powerball_cart_controller_ns::PowerballCartController,
                         pr2_controller_interface::Controller)

