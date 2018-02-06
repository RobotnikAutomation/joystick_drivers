/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

/*
 * Author: Stuart Glaser
 */

#include <stdio.h>
#include <vector>
#include "ros/node_handle.h"
#include "ros/param.h"
#include "spnav.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <robotnik_msgs/enable_disable.h>

#define DEFAULT_SCALE_LINEAR	1.0
#define DEFAULT_SCALE_ANGULAR	2.0
#define DEFAULT_THRESHOLD     0.1

//! Flag to enable/disable the communication with the publishers topics
bool bEnable = true; // Communication flag enabled by default
double threshold_pad = DEFAULT_THRESHOLD;

/**
 * Return the sign
*/
float sign(float x){
  if(x < 0.0){
    return -1.0;
  }
  return 1.0;
}

/**
 * Checks whether the value is lower or bigger than a threshold
*/
double check_threshold(double value){
  if(fabs(value) < threshold_pad){
    return 0.0;
  }
  return sign(value) * (fabs(value - threshold_pad));
}

/**
 *  Service than enables/disables the spacenav
 * 
*/
bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res){
  bEnable = req.value;
  ROS_INFO("SummitXL::Spacenav: Setting to %d", req.value);
  res.ret = true;
  return true;		
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacenav");
  ros::NodeHandle private_nh("~");

  ros::NodeHandle node_handle;
  ros::Publisher offset_pub = private_nh.advertise<geometry_msgs::Vector3>("offset", 2);
  ros::Publisher rot_offset_pub = private_nh.advertise<geometry_msgs::Vector3>("rot_offset", 2);
  ros::Publisher twist_pub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);
  ros::Publisher joy_pub = private_nh.advertise<sensor_msgs::Joy>("joy", 2);

  ros::ServiceServer enable_disable_srv = private_nh.advertiseService("enable_disable",  EnableDisable);

  // Used to scale joystick output to be in [-1, 1]. Estimated from data, and not necessarily correct.
 
  double full_scale;
  private_nh.param<double>("full_scale", full_scale, 512);
  if (full_scale < 1e-10)
  {
    full_scale = 512;
  }
  // Scale factors for the different axes. End output will be within [-scale, +scale], provided
  // full_scale normalizes to within [-1, 1].
  double linear_scale;
  double angular_scale;
  double current_vel;
  double control_speed = 0.0;
  double target_speed = 0.0;

  private_nh.param<double>("linear_scale", linear_scale, DEFAULT_SCALE_LINEAR);
  private_nh.param<double>("angular_scale", angular_scale, DEFAULT_SCALE_ANGULAR);
  private_nh.param<double>("current_vel", current_vel, current_vel);
  private_nh.param<double>("threshold_pad", threshold_pad, threshold_pad);

  //ROS_INFO("linear_scale: %.3f", linear_scale);
  //ROS_INFO("angular_scale: %.3f", angular_scale);

  if (spnav_open() == -1)
  {
    ROS_ERROR("Could not open the space navigator device.  Did you remember to run spacenavd (as root)?");
    return 1;
  }

  // Parameters
  // The number of polls needed to be done before the device is considered "static"
  int static_count_threshold = 30;
  ros::param::get("~/static_count_threshold",static_count_threshold);

  // If true, the node will zero the output when the device is "static"
  bool zero_when_static = true;
  ros::param::get("~/zero_when_static",zero_when_static);

  // If the device is considered "static" and each trans, rot normed component
  // is below the deadband, it will output zeros in either rotation,
  // translation, or both.
  double static_trans_deadband = 0.1; 
  double static_rot_deadband = 0.1;
  ros::param::get("~/static_trans_deadband", static_trans_deadband);
  ros::param::get("~/static_rot_deadband", static_rot_deadband);

  sensor_msgs::Joy joystick_msg;
  joystick_msg.axes.resize(6);
  joystick_msg.buttons.resize(2);
  
  spnav_event sev;
  int no_motion_count = 0;
  bool motion_stale = false;
  double normed_vx = 0;
  double normed_vy = 0;
  double normed_vz = 0;
  double normed_wx = 0;
  double normed_wy = 0;
  double normed_wz = 0;
  while (ros::ok())
  {
    if(bEnable){
      bool joy_stale = false;
      bool queue_empty = false;
      
      // Sleep when the queue is empty.
      // If the queue is empty 30 times in a row output zeros.
      // Output changes each time a button event happens, or when a motion
      // event happens and the queue is empty.
      joystick_msg.header.stamp = ros::Time().now();
      switch (spnav_poll_event(&sev))
      {
        case 0:
          queue_empty = true;
          if (++no_motion_count > static_count_threshold)
          {
            if ( zero_when_static || 
                ( fabs(normed_vx) < static_trans_deadband &&
                  fabs(normed_vy) < static_trans_deadband &&
                  fabs(normed_vz) < static_trans_deadband)
              )
            {
              normed_vx = normed_vy = normed_vz = 0;
            }

            if ( zero_when_static || 
                ( fabs(normed_wx) < static_rot_deadband &&
                  fabs(normed_wy) < static_rot_deadband &&
                  fabs(normed_wz) < static_rot_deadband )
              )
            {
              normed_wx = normed_wy = normed_wz = 0;
            }

            no_motion_count = 0;
            motion_stale = true;
          }
          break;

        case SPNAV_EVENT_MOTION:
          normed_vx = sev.motion.z / full_scale;
          normed_vx = check_threshold(normed_vx);

          normed_vy = -sev.motion.x / full_scale;
          normed_vy = check_threshold(normed_vy);

          normed_vz = sev.motion.y / full_scale;
          normed_vz = check_threshold(normed_vz);

          normed_wx = sev.motion.rz / full_scale;
          normed_wx = check_threshold(normed_wx);

          normed_wy = -sev.motion.rx / full_scale;
          normed_wy = check_threshold(normed_wy);

          normed_wz = sev.motion.ry / full_scale;
          normed_wz = check_threshold(normed_wz);

          motion_stale = true;
          break;
          
        case SPNAV_EVENT_BUTTON:
          //printf("type, press, bnum = <%d, %d, %d>\n", sev.button.type, sev.button.press, sev.button.bnum);
          joystick_msg.buttons[sev.button.bnum] = sev.button.press;

          joy_stale = true;
          break;

        default:
          ROS_WARN("Unknown message type in spacenav. This should never happen.");
          break;
      }
    
      if (motion_stale && (queue_empty || joy_stale))
      {
        // The offset and rot_offset are scaled.
        geometry_msgs::Vector3 offset_msg;
        offset_msg.x = normed_vx * linear_scale * current_vel;
        offset_msg.y = normed_vy * linear_scale * current_vel;
        offset_msg.z = normed_vz * linear_scale * current_vel;
        offset_pub.publish(offset_msg);

        geometry_msgs::Vector3 rot_offset_msg;
        rot_offset_msg.x = normed_wx * angular_scale * current_vel;
        rot_offset_msg.y = normed_wy * angular_scale * current_vel;
        rot_offset_msg.z = normed_wz * angular_scale * current_vel;
        rot_offset_pub.publish(rot_offset_msg);

        geometry_msgs::Vector3 linear_msg;
        // Select only the biggest value
        if(fabs(normed_wx) > fabs(normed_wy) && fabs(normed_wx) > fabs(normed_wz)){
          linear_msg.y = -(normed_wx * linear_scale * 0.2);
          linear_msg.x = 0.0;
          linear_msg.z = 0.0;
        }else if(fabs(normed_wy) > fabs(normed_wx) && fabs(normed_wy) > fabs(normed_wz)){
          linear_msg.y = 0.0;
          linear_msg.x = normed_wy * linear_scale * current_vel;
          if(linear_msg.x > 0){
            linear_msg.x *= 2.8;
          }
          linear_msg.z = 0.0;
        }else{
          linear_msg.x = 0.0;
          linear_msg.y = 0.0;
          linear_msg.z = normed_wz * linear_scale * current_vel;
        }
        
        // Velocity smooth
        if(linear_msg.x > control_speed){
          control_speed = std::min(linear_msg.x, control_speed + 0.004);
        }else if(linear_msg.x < control_speed){
          control_speed = std::max(linear_msg.x, control_speed - 0.006);
        }else{
          control_speed = linear_msg.x;
        }
        linear_msg.x = control_speed;


        // Publish linear and angular velocity
        geometry_msgs::Twist twist_msg;
        twist_msg.linear = linear_msg;
        twist_msg.angular = rot_offset_msg;
        twist_pub.publish(twist_msg);

        joystick_msg.axes[0] = normed_vx;
        joystick_msg.axes[1] = normed_vy;
        joystick_msg.axes[2] = normed_vz;
        joystick_msg.axes[3] = normed_wx;
        joystick_msg.axes[4] = normed_wy;
        joystick_msg.axes[5] = normed_wz;

        no_motion_count = 0;
        motion_stale = false;
        joy_stale = true;
      }
    
      if (joy_stale)
      {
        joy_pub.publish(joystick_msg);
      }

      if (queue_empty) {
        usleep(1000);
      }

    }
    ros::spinOnce();
  }

  spnav_close();

  return 0;
}
