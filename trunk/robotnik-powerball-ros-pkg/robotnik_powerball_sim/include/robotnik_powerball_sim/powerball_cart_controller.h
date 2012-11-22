/* powerball_cart_controller.h
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


#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <boost/scoped_ptr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp> 
#include <robotnik_powerball_pad/ArmRef.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>
#include <planning_models/kinematic_state.h>

#define CARTESIAN_MODE 	0
#define EULER_MODE		1
#define JBJ_TELEOP_MODE 2


namespace powerball_cart_controller_ns{

class PowerballCartController: public pr2_controller_interface::Controller
{
private:
  // The current joint state                                                                                                                             
  pr2_mechanism_model::JointState* joint_state_;

  // The chain of links and joints                                                                                                                                                
  pr2_mechanism_model::Chain chain_;
  
  KDL::JntArray jnt_pos_, jnt_effort_, jnt_ref_vel_, ev_, jnt_ref_pos_;
  KDL::JntArrayVel jnt_real_vel_;
  KDL::Frame reference_pose_;
  KDL::Twist incr_pos_;
  
  trajectory_msgs::JointTrajectoryPoint pt_ant;
  
  //planning_environment::CollisionModels collision_models;
  boost::scoped_ptr<planning_environment::CollisionModels> collision_models_;

  // KDL Solvers performing the actual computations                                                                                                                               
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  //boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> jnt_to_vel_solver_;

  // Subscriber
  ros::Subscriber arm_ref_sub_;
  // Publishers
  ros::Publisher arm_com_pub_;
  ros::Publisher joint_positions_pub_;


  double prx_, pry_, prz_;                                                                                                                 

  KDL::Vector last_position_;
  
  bool check_self_collision, check_joint_limits;
  bool no_self_collision;
  
  // Joint
  std::vector<std::string> joint_names;
  
  // Joint limits
  double joint_limits[6];
  char actualJoint_;
  int dp;
  
  // Operation Mode (Cartesian/Euler/JbyJ)
  char opMode;

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();
  void refCallback(const robotnik_powerball_pad::ArmRef::ConstPtr& msg);  
};
}
