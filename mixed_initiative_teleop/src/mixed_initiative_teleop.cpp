/*!
 * mixed_initiative_teleop_node.cpp
 * Copyright (c) 2014, Manolis Chiou
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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
 */

/*!

@mainpage
  Joystick teleoperation node for use within the mixed initiative framework. The user can operate the robot in
  teleoperation mode and change on the fly autonomy level/mode. Also a stop button is implimented.
  It was ment to be used with an Xbox 360 joystick but should work with any joystick.
<hr>

@section usage Usage
@par    After start roscore, you need load robot configuration file to parameter server first.
        For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
        then run drrobot_player first. then start ros joy node.
@verbatim
$ mixed_initiative_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b /teleop/cmd_vel: will publish to /teleop/cmd_vel a geometry_msgs/Twist.msg type message to drrobot_player.
 For differential robots, linear.x is forward/backward speed (m/sec), and angular.z (rad/sec)is the angular speed.
<hr>
*/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

class JoystickTeleop
{

public:
    JoystickTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_axis_, angular_axis_, control_button_, stop_button_, auto_button_, teleop_button_ , bell_button_ ,ref_button_,doctor_button_,doctorrec_button_;
    int up_button_, down_button_, left_button_, right_button_;
    double linear_scaling_, angular_scaling_;

    ros::Publisher vel_pub_, mode_pub_ , bell_pub_ ,person_pub_,doctor_pub_, recognizer_pub_;
ros::Publisher cancelGoal_pub_;
std_msgs::Empty cancelGoal_;

    ros::Subscriber joy_sub_;

};


JoystickTeleop::JoystickTeleop()
{
    // Default movement axis
    nh_.param("axis_linear", linear_axis_, 1);
    nh_.param("axis_angular", angular_axis_, 0);

    // Default scaling parameters
    nh_.param("scale_angular", angular_scaling_, 1.0);
    nh_.param("scale_linear", linear_scaling_, 0.5);

    //Default buttons for Xbox 360 joystick.
    nh_.param("teleop_button", teleop_button_, 3); // Y button
    nh_.param("stop_button", stop_button_, 1);     // B button
    nh_.param("auto_button", auto_button_, 0);     // A button
    nh_.param("bell_button", bell_button_, 2);     // X button
    nh_.param("ref_button", ref_button_, 4);       // left bumper button
    nh_.param("doctor_button", doctor_button_, 5); // Right bumper button (for Doctor leaving room)
    nh_.param("up_button", up_button_, 13);     // up in digital cross type: 2 state:1 (task2)
    nh_.param("down_button", down_button_, 14);     // down in digital cross type: 2 state:2 (task2)
    nh_.param("left_button", left_button_, 11);     // left in digital cross type: 1 state:1 (task1)
    nh_.param("right_button", right_button_, 12);     // right in digital cross type: 1 state:2 (task1)
    nh_.param("doctorrec_button", doctorrec_button_, 6); //to trigger that the doctor was recognise


 cancelGoal_pub_ = nh_.advertise<std_msgs::Empty>("/navigation/stop", 5,this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 5);
    mode_pub_ = nh_.advertise<std_msgs::Int8>("/control_mode", 5);
    bell_pub_ = nh_.advertise<std_msgs::Empty>("/roah_rsbb/devices/bell" , 5);
    person_pub_ = nh_.advertise<std_msgs::String>("/recognition/response" , 5);
    doctor_pub_ = nh_.advertise<std_msgs::String>("/is_human_in_region_output" , 5);
    //added by lenka to overwrite recognition
    recognizer_pub_ = nh_.advertise<std_msgs::String>("/recognition/response",5);
    

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickTeleop::joyCallback, this);

}

void JoystickTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist cmd_vel;
    std_msgs::Empty bell_msg;
    std_msgs::Int8 mode;
    std_msgs::String doctor_wait;
    std_msgs::String Doctor;
    Doctor.data = "Doctor";


    // movement commands
    cmd_vel.linear.x = linear_scaling_ * joy->axes[linear_axis_];
    cmd_vel.angular.z = angular_scaling_ * joy->axes[angular_axis_];

    vel_pub_.publish(cmd_vel);

    // autonomy mode choice
    if (joy->buttons[stop_button_] && !joy->buttons[ref_button_])
    {
        mode.data=0;
        mode_pub_.publish(mode);
    }
    else if (joy->buttons[teleop_button_] && !joy->buttons[ref_button_])
    {
        mode.data=1;
        mode_pub_.publish(mode);
    }
    else if (joy->buttons[auto_button_] && !joy->buttons[ref_button_])
    {
        mode.data=2;
        mode_pub_.publish(mode);
    }

    // leg detection overide
    else if (joy->buttons[doctor_button_] && !joy->buttons[ref_button_])
    {
        doctor_pub_.publish(doctor_wait);
    }

    // recognizer overide
    else if (joy->buttons[doctorrec_button_] && !joy->buttons[ref_button_])
    {
        recognizer_pub_.publish(Doctor);
    }


    // simulate bell msg
    else if (joy->buttons[bell_button_] && !joy->buttons[ref_button_])
    {
        bell_pub_.publish(bell_msg);
	 cancelGoal_pub_.publish(cancelGoal_);
    }

    // overide flat
    else if (joy->buttons[up_button_] && !joy->buttons[ref_button_]) // task 1
    {
        system("rosservice call /roah_rsbb/override \"{benchmark_type: 2, benchmark_state: 1}\"");
	ROS_INFO("UP \"{benchmark_type: 2, benchmark_state: 1}\"");
    }

    else if (joy->buttons[down_button_] && !joy->buttons[ref_button_])
    {
        system("rosservice call /roah_rsbb/override \"{benchmark_type: 2, benchmark_state: 2}\"");
	ROS_INFO("DOWN");
    }

    else if (joy->buttons[left_button_] && !joy->buttons[ref_button_]) // task 2
    {
        system("rosservice call /roah_rsbb/override \"{benchmark_type: 1, benchmark_state: 1}\"");
	ROS_INFO("LEFT");
    }

    else if (joy->buttons[right_button_] && !joy->buttons[ref_button_])
    {
        system("rosservice call /roah_rsbb/override \"{benchmark_type: 1, benchmark_state: 2}\"");
	ROS_INFO("right");
    }



    // When the ref pushes the left bumber button it can choose between persons
    else if (joy->buttons[ref_button_])
    {
        // B button
        if (joy->buttons[stop_button_])
        {
            std_msgs::String response ;
            response.data = "Doctor" ;
            person_pub_.publish(response);
        }
        // Y button
        else if (joy->buttons[teleop_button_])
        {
            std_msgs::String response  ;
            response.data = "Deliman" ;
            person_pub_.publish(response);
        }
        // A button
        else if (joy->buttons[auto_button_])
        {
            std_msgs::String response  ;
            response.data = "Postman" ;
            person_pub_.publish(response);
        }
        // X button
        else if (joy->buttons[bell_button_])
        {
            std_msgs::String response  ;
            response.data = "Unknown" ;
            person_pub_.publish(response);
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mixed_initiative_teleop");
    JoystickTeleop joystick_teleop;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
