/*
 * robotnik_powerball_pad
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
 * \author Robotnik Automation SLL
 * \brief Allows to use a pad with robotnik_powerball_driver.
 */

//#define WSG_50_GRIPPER  	// Uncomment this line if you are using the Powerball with a WSG 50 Gripper
#define PG_70_GRIPPER	    // Uncomment this line if you are using the Powerball with a PG+ 70 Gripper

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#if (defined WSG_50_GRIPPER)
	#include <wsg_50_common/Move.h>
	#include <wsg_50_common/Incr.h>
#endif
#include <robotnik_powerball_pad/ArmRef.h>
#if (defined PG_70_GRIPPER)
	#include <robotnik_pg70_driver/Move.h>
	#include <robotnik_pg70_driver/MoveIncr.h>
#endif

#define DEFAULT_NUM_OF_BUTTONS		20
#define ARM							0
#define GRIPPER						1


class PowerballPad
{
	public:
	PowerballPad();

	private:
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;


	double l_scale_;
	//! It will publish into command velocity (for the robot)
	ros::Publisher arm_ref_pub_, gripper_ref_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;


	//! Number of the button actions
	int button_up_open_, button_down_close_;
	int button_aux_;
	int button_fold_;
	int button_select_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	//! buttons to the gripper
	int dead_man_gripper_;
	//! buttons to the arm
	int dead_man_arm_;
	//! current connection (0: arm, 1: gripper)
	short deviceConnected;
	//! Service to move the gripper/arm
	ros::ServiceClient gripper_setOperationMode_client;
	ros::ServiceClient gripper_move_client;
	ros::ServiceClient gripper_move_incr_client;
	ros::ServiceClient gripper_close_client;
	
	ros::ServiceClient arm_setOperationMode_client;
	ros::ServiceClient arm_fold_client;
	ros::ServiceClient arm_close_client;
	
};


PowerballPad::PowerballPad(){


	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

	//GENERIC BUTTONS
	nh_.param("button_up_open", button_up_open_, button_up_open_);  			// Triangle PS3
	nh_.param("button_down_close", button_down_close_, button_down_close_); 	// Cross PS3
	nh_.param("button_aux", button_aux_, button_aux_);							// Circle PS3
	nh_.param("button_fold", button_fold_, button_fold_);						// Square PS3

	// GRIPPER CONF
	nh_.param("dead_man_gripper", dead_man_gripper_, dead_man_gripper_);		// L1 PS3

	// ARM CONF
	nh_.param("dead_man_arm", dead_man_arm_, dead_man_arm_);					// R2 PS3
	nh_.param("button_select", button_select_, button_select_);					// Select PS3 (Joint teleop. => Cartesian/Euler teleop. => Joint traj.)

	//bRegisteredButtonEvent = new bool(num_of_buttons_);
	for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PowerballPad::padCallback, this);

	// Request service to send commands to the gripper/arm
	#if (defined WSG_50_GRIPPER)
	gripper_move_client = nh_.serviceClient<wsg_50_common::Incr>("/wsg_50/move_incrementally");
	gripper_grasp_client = nh_.serviceClient<wsg_50_common::Move>("/wsg_50/grasp");
	#endif
	#if (defined PG_70_GRIPPER)
	gripper_move_incr_client = nh_.serviceClient<robotnik_pg70_driver::MoveIncr>("/robotnik_pg70_driver/move_incrementally");
	gripper_move_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_pg70_driver/open");
	gripper_close_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_pg70_driver/close_can_comm");
	gripper_setOperationMode_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_pg70_driver/set_operation_mode");
	#endif
	arm_setOperationMode_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_powerball_driver/set_operation_mode");
	arm_fold_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_powerball_driver/fold");
	arm_close_client = nh_.serviceClient<std_srvs::Empty>("/robotnik_powerball_driver/close_can_comm");

	//Publishes into the arm controller
	arm_ref_pub_ = nh_.advertise<robotnik_powerball_pad::ArmRef>("/robotnik_powerball_pad/arm_reference", 1);
	#if (defined PG_70_GRIPPER)
	gripper_ref_pub_ = nh_.advertise<robotnik_powerball_pad::ArmRef>("/robotnik_pg70_driver/finger_reference", 1);
	#endif
	
}

void PowerballPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	robotnik_powerball_pad::ArmRef arm;
	robotnik_powerball_pad::ArmRef finger;


	#if (defined WSG_50_GRIPPER)
	// WSG50 GRIPPER MOVEMENTS

  	// Actions dependant on gripper dead-man button
 	if (joy->buttons[dead_man_gripper_] == 1) {
		if (joy->buttons[button_up_open_] == 1){
			if(!bRegisteredButtonEvent[button_up_open_]){
				//ROS_ERROR("OPEN GRIPPER");
				wsg_50_common::Incr incr_srv;
				incr_srv.request.direction = "open"; 
				incr_srv.request.increment = 10;
				gripper_move_client.call(incr_srv);
				bRegisteredButtonEvent[button_up_open_] = true;
			}
		}else if (joy->buttons[button_down_close_] == 1){
			if(!bRegisteredButtonEvent[button_down_close_]){
				//ROS_ERROR("CLOSE GRIPPER");
				wsg_50_common::Incr incr_srv;
				incr_srv.request.direction = "close"; 
				incr_srv.request.increment = 10;
				gripper_move_client.call(incr_srv);
				bRegisteredButtonEvent[button_down_close_] = true;
			}
		}else if (joy->buttons[button_aux_] == 1){
			if(!bRegisteredButtonEvent[button_aux_]){
				//ROS_ERROR("GRASP WITH GRIPPER");
				wsg_50_common::Move grasp_srv;
				grasp_srv.request.width = 0.0; 
				grasp_srv.request.speed = 10.0;
				gripper_grasp_client.call(grasp_srv);
				bRegisteredButtonEvent[button_aux_] = true;
			}
		}else{
			bRegisteredButtonEvent[button_up_open_] = false;
			bRegisteredButtonEvent[button_down_close_] = false;
			bRegisteredButtonEvent[button_aux_] = false;
		}
		
	}
	#endif
	
	#if (defined PG_70_GRIPPER)
	// PG+ 70 GRIPPER MOVEMENTS

  	// Actions dependant on gripper dead-man button
 	if (joy->buttons[dead_man_gripper_] == 1 && joy->buttons[dead_man_arm_] == 0) {
		if (joy->buttons[button_up_open_] == 1){
			if(!bRegisteredButtonEvent[button_up_open_]){
				//ROS_ERROR("OPEN GRIPPER");
				//robotnik_pg70_driver::MoveIncr incr_srv;
				//incr_srv.request.direction = "open"; 
				//incr_srv.request.increment = 10000;
				//gripper_move_incr_client.call(incr_srv);
				
				
				//robotnik_pg70_driver::MoveIncr incr_srv;
				//gripper_move_client.call(move_srv);
				
				std_srvs::Empty srv_empty;
				gripper_move_client.call(srv_empty);
				
				bRegisteredButtonEvent[button_up_open_] = true;
			}
		/*	
		}else if (joy->buttons[button_down_close_] == 1){
			if(!bRegisteredButtonEvent[button_down_close_]){
				//ROS_ERROR("CLOSE GRIPPER");
				robotnik_pg70_driver::MoveIncr incr_srv;
				incr_srv.request.direction = "close"; 
				incr_srv.request.increment = 10000;
				gripper_move_incr_client.call(incr_srv);
				bRegisteredButtonEvent[button_down_close_] = true;
			}
		*/ 
		}else if (joy->buttons[button_aux_] == 1){
			if(!bRegisteredButtonEvent[button_aux_]){
				finger.aux_reference = 1;
				bRegisteredButtonEvent[button_aux_] = true;
			}
		}else{
			
			finger.x_position_reference = joy->axes[1];
			
			bRegisteredButtonEvent[button_up_open_] = false;
			bRegisteredButtonEvent[button_down_close_] = false;
			bRegisteredButtonEvent[button_aux_] = false;
		}
		
	}
	
	gripper_ref_pub_.publish(finger);
	#endif
	
	
	
	// ARM MOVEMENTS
	
	// Actions dependant on arm dead-man button
	if (joy->buttons[dead_man_arm_] == 1 && joy->buttons[dead_man_gripper_] == 0){
		
		if (joy->buttons[button_up_open_] == 1){
			if(!bRegisteredButtonEvent[button_up_open_]){

				bRegisteredButtonEvent[button_up_open_] = true;
				arm.joint_reference = 1;
					
			}
		}else if (joy->buttons[button_down_close_] == 1){
			if(!bRegisteredButtonEvent[button_down_close_]){

				bRegisteredButtonEvent[button_down_close_] = true;
				arm.joint_reference = -1;
					
			}
		// Allow to change between slow and fast speed in joint-by-joint teleoperation mode 
		// and between cartesian/euler in this mode.
		}else if (joy->buttons[button_aux_] == 1){
			if(!bRegisteredButtonEvent[button_aux_]){

				bRegisteredButtonEvent[button_aux_] = true;
				arm.aux_reference = 1;
					
			}
			
		// Allow selection between the three possible operation modes
		}else if (joy->buttons[button_select_] == 1){
			if(!bRegisteredButtonEvent[button_select_]){
				bRegisteredButtonEvent[button_select_] = true;
				std_srvs::Empty srv;
				arm_setOperationMode_client.call(srv);
			}
			
		// Allow to fold the arm throught the pad	
		}else if (joy->buttons[button_fold_] == 1){
			if(!bRegisteredButtonEvent[button_fold_]){
				bRegisteredButtonEvent[button_fold_] = true;
				std_srvs::Empty srv;
				arm_fold_client.call(srv);
		
			}
		}else{
			
			arm.x_position_reference = joy->axes[1];
			arm.y_position_reference = joy->axes[0];
			arm.z_position_reference = joy->axes[3];
			
			bRegisteredButtonEvent[button_up_open_] = false;
			bRegisteredButtonEvent[button_down_close_] = false;
			bRegisteredButtonEvent[button_aux_] = false;
			bRegisteredButtonEvent[button_select_] = false;
			bRegisteredButtonEvent[button_fold_] = false;
		}
	}
	
	arm_ref_pub_.publish(arm);
	
	if(joy->buttons[dead_man_gripper_] == 1 && joy->buttons[dead_man_arm_] == 1){
	
		if (joy->buttons[button_select_] == 1){
			if(!bRegisteredButtonEvent[button_select_]){
				bRegisteredButtonEvent[button_select_] = true;
				std_srvs::Empty srv;
				if (deviceConnected == ARM){
					arm_close_client.call(srv);
					gripper_setOperationMode_client.call(srv);
					ROS_ERROR("ARM: DISABLED // GRIPPER: ENABLED");
					deviceConnected = GRIPPER;
				}else if (deviceConnected == GRIPPER){
					gripper_close_client.call(srv);
					arm_setOperationMode_client.call(srv);
					ROS_ERROR("ARM: ENABLED // GRIPPER: DISABLED");
					deviceConnected = ARM;
				}
			}
		}else{
			bRegisteredButtonEvent[button_select_] = false;
		}
		
	}
	
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_powerball_pad");
	PowerballPad robotnik_powerball_pad;
	ros::spin();
}

