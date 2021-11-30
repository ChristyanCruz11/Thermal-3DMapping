//=============================================================================
// This program is a modification of 
//                  $(find hector_quadrotor_teleop)/src/quadrotor_teleop.cpp
// Editor  : Masaru Shimizu
// E-mail  : shimizu@sist.chukyo-u.ac.jp
// Updated : 25 Mar.2018
//=============================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=============================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <list>

namespace teleop_jaguar
{

class Teleop
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber,
                  stairs_distance_subscriber,
                  front_distance_subscriber,
                  floor_distance_subscriber;
  ros::Publisher  flipper_publisher,
                  velocity_publisher;
  std::string     topicname_cmd_flipper,
                  topicname_cmd_vel,
                  topicname_joy,
                  framename_base_link,
                  framename_world,
                  topicname_stairs_distance,
                  topicname_front_distance,
                  topicname_floor_distance;
  bool  stairs_reached,
        isClimbing,
        descend_stairs;
  int   start_climb_button,
        floor_distance_peak_count,
        floor_distance_peak_marker;

  double floor_distance_peak,
         floor_distance_peak_mean,
         climbing_safe_factor,
         descend_safe_factor;

  std::list<double> peak_list;

  struct Axis
  {
    Axis(const std::string& _name)
      : axis(0), factor(0.0), offset(0.0), name(_name)
    {}
    int axis;
    double factor;
    double offset;
    std::string name;
  };

  struct Velocity
  {
    Velocity()
      : speed("Speed"), turn("Turn")
    {}
    Axis speed, turn;
  } velocity;

  struct Flipper
  {
    Flipper(const std::string& _name) 
     : w(5.0), currentAngle(0.0), defaultAngle(M_PI/4.0), 
       upButton(0), downButton(0), name(_name) 
    {}
    int    upButton, downButton;
    double w;
    double currentAngle;
    double defaultAngle;
    std::string name;
  };

  struct Flippers 
  {
    Flippers()
      : fr("fr"), fl("fl"), rr("rr"), rl("rl")
    {}
    Flipper fr, fl, rr, rl;
  } flipper;

public:
  void Usage(void)
  {
    printf("==================== Joystick Usage ====================\n");
    printf("\n");
    printf(" Robot Moving   : The left analog stick\n");
    printf("--------------------------------------------------------\n");
    printf(" Flipper Moving : (followings are number of button)\n");
    printf("    Front Right : up=%d , down=%d\n"
                             , flipper.fr.upButton+1, flipper.fr.downButton+1);
    printf("    Front Left  : up=%d , down=%d\n"
                             , flipper.fl.upButton+1, flipper.fl.downButton+1);
    printf("    Rear  Right : up=%d , down=%d\n"
                             , flipper.rr.upButton+1, flipper.rr.downButton+1);
    printf("    Rear  Left  : up=%d , down=%d\n"
                             , flipper.rl.upButton+1, flipper.rl.downButton+1);
    printf("\n");
    printf("========================================================\n");    
  }
  
  Teleop()
  {
    // Make a nodehandle for reading parameters from the local namespace.
    ros::NodeHandle _nh("~");
    // TODO Read Topicnames and framenames
    _nh.param<std::string>("topicname_stairs_distance", topicname_stairs_distance, "stairs_distance");
    _nh.param<std::string>("topicname_front_distance", topicname_front_distance, "front_distance");
    _nh.param<std::string>("topicname_floor_distance", topicname_floor_distance, "floor_distance");
    _nh.param<std::string>("topicNameJoy",        topicname_joy, "joy");
    _nh.param<std::string>("frameNameWorld",      framename_world,"world");
    _nh.param<std::string>("frameNameBaselink",   framename_base_link, 
                                                            "base_link");
    _nh.param<std::string>("topicNameCmdflipper", topicname_cmd_flipper,
                                                          "cmd_flipper");
    _nh.param<std::string>("topicNameCmdvel", topicname_cmd_vel,
                                                          "cmd_vel");
    _nh.param<int>("start_climb_button", start_climb_button, 9);
    _nh.param<double>("climbing_safe_factor", climbing_safe_factor, 1.2);
    _nh.param<double>("descend_safe_factor", descend_safe_factor, 4.0);

    // Read parameters for structure velocity
    _nh.param<int>(   "speedAxis",      velocity.speed.axis, 1);
    _nh.param<int>(   "turnAxis",       velocity.turn.axis, 0);
    _nh.param<double>("maxSpeed",       velocity.speed.factor, .5);
    _nh.param<double>("maxTurnW",       velocity.turn.factor, 2.0*M_PI/4.0);
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "frUpButton",     flipper.fr.upButton, 0);
    _nh.param<int>(   "frDownButton",   flipper.fr.downButton, 2);
    _nh.param<double>("frW",            flipper.fr.w, 2.0*M_PI/20.0);
    _nh.param<double>("frDefaultAngle", flipper.fr.defaultAngle, M_PI/4.0);
    flipper.fr.currentAngle = flipper.fr.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "flUpButton",     flipper.fl.upButton, 0);
    _nh.param<int>(   "flDownButton",   flipper.fl.downButton, 2);
    _nh.param<double>("flW",            flipper.fl.w, 2.0*M_PI/20.0);
    _nh.param<double>("flDefaultAngle", flipper.fl.defaultAngle, M_PI/4.0);
    flipper.fl.currentAngle = flipper.fl.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "rrUpButton",     flipper.rr.upButton, 15);
    _nh.param<int>(   "rrDownButton",   flipper.rr.downButton, 16);
    _nh.param<double>("rrW",            flipper.rr.w, 2.0*M_PI/20.0);
    _nh.param<double>("rrDefaultAngle", flipper.rr.defaultAngle, M_PI/4.0);
    flipper.rr.currentAngle = flipper.rr.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "rlUpButton",     flipper.rl.upButton, 15);
    _nh.param<int>(   "rlDownButton",   flipper.rl.downButton, 16);
    _nh.param<double>("rlW",            flipper.rl.w, 2.0*M_PI/20.0);
    _nh.param<double>("rlDefaultAngle", flipper.rl.defaultAngle, M_PI/4.0);
    flipper.rl.currentAngle = flipper.rl.defaultAngle;

    joy_subscriber = node_handle_.subscribe<sensor_msgs::Joy>(topicname_joy, 1,
                       boost::bind(&Teleop::joyCallback, this, _1));
    stairs_distance_subscriber = node_handle_.subscribe<std_msgs::Int32>(topicname_stairs_distance, 1,
                       boost::bind(&Teleop::stairsCallback, this, _1));
    front_distance_subscriber = node_handle_.subscribe<sensor_msgs::Range>(topicname_front_distance, 1,
                       boost::bind(&Teleop::frontCallback, this, _1));
    floor_distance_subscriber = node_handle_.subscribe<sensor_msgs::Range>(topicname_floor_distance, 1,
                       boost::bind(&Teleop::floorCallback, this, _1));
    velocity_publisher = node_handle_.advertise<geometry_msgs::TwistStamped>
                           (topicname_cmd_vel, 10);
    flipper_publisher  = node_handle_.advertise<geometry_msgs::TwistStamped>
                           (topicname_cmd_flipper, 10);

    this->isClimbing = false;
  }

  ~Teleop()
  {
    stop();
  }

  double updateCurrentFlipperAngle(Flipper& flpr, 
                                   const sensor_msgs::JoyConstPtr& joy)
  {
    if(getUpButton(joy, flpr) && getDownButton(joy, flpr))
      flpr.currentAngle = flpr.defaultAngle;
    else if(getUpButton(joy, flpr))
      flpr.currentAngle += flpr.w;
    else if(getDownButton(joy, flpr))
      flpr.currentAngle -= flpr.w;
    return flpr.currentAngle;
  }

  double updateCurrentFlipperAngle(Flipper& flpr, double angle)
  {
    flpr.currentAngle = angle;
    return flpr.currentAngle;
  }

  bool check_climb_start(const sensor_msgs::JoyConstPtr& joy)
  {
    if(this->stairs_reached)
    {
      if(joy->buttons[this->start_climb_button] > 0)
      {
        return true;
      }
    }
    return false;
  }

  void publish_flipper(void)
  {
    geometry_msgs::TwistStamped flp_ts;
    flp_ts.header.frame_id = framename_base_link;
    flp_ts.header.stamp = ros::Time::now();
    flp_ts.twist.linear.x  = flipper.fr.currentAngle;
    flp_ts.twist.linear.y  = flipper.fl.currentAngle;
    flp_ts.twist.linear.z  = 0;
    flp_ts.twist.angular.x = flipper.rr.currentAngle;
    flp_ts.twist.angular.y = flipper.rl.currentAngle;
    flp_ts.twist.angular.z = 0;
    flipper_publisher.publish(flp_ts);
/*MEMO
    geometry_msgs::Twist flp_ts;
    flp_ts.linear.x  = flipper.fr.currentAngle;
    flp_ts.linear.y  = flipper.fl.currentAngle;
    flp_ts.linear.z  = 0;
    flp_ts.angular.x = flipper.rr.currentAngle;
    flp_ts.angular.y = flipper.rl.currentAngle;
    flp_ts.angular.z = 0;
    flipper_publisher.publish(flp_ts);
*/
  }

  void frontCallback(const sensor_msgs::RangeConstPtr &rng)
  {
    if(rng->range < 0.06)
      this->stairs_reached = true;
    else
      this->stairs_reached = false;
  }

  void stairsCallback(const std_msgs::Int32::ConstPtr& distance)
  {
    if(distance->data > 100 && !this->stairs_reached)
      ROS_INFO("Distance YES");
    else
      if(this->stairs_reached)
        ROS_INFO("READY TO CLIMB");
  }

  void floorCallback(const sensor_msgs::RangeConstPtr &rng)
  {
    if(this->isClimbing)
    {
      if(double(rng->range) > this->floor_distance_peak)
      {
        this->floor_distance_peak = double(rng->range);
        this->floor_distance_peak_marker = this->floor_distance_peak_count;
        //ROS_INFO("New PEAK: %lf in count:%d", this->floor_distance_peak, this->floor_distance_peak_marker);
      }
      if(this->floor_distance_peak_count - this->floor_distance_peak_marker > 10)
      {
        this->peak_list.push_back(this->floor_distance_peak);
        double avg = 0;
        std::list<double>::iterator it;
        for(it = this->peak_list.begin(); it != this->peak_list.end(); it++) 
        {
          avg += *it;
        }
        avg /= this->peak_list.size();
        this->floor_distance_peak_mean = avg;
        if(this->peak_list.size() > 30){
          this->peak_list.pop_front();
        }
        //ROS_INFO("AVG %lf in count:%d", this->floor_distance_peak_mean, this->floor_distance_peak_count);
      }
      if(double(rng->range) > this->climbing_safe_factor * this->floor_distance_peak_mean && this->floor_distance_peak_count > 30 && this->floor_distance_peak_mean > 0)
        {
          stop();
        }
      this->floor_distance_peak_count += 1;
    }
    else
    {
      if(double(rng->range) > this->descend_safe_factor * this->floor_distance_peak_mean && this->floor_distance_peak_mean > 0)
        {
          this->descend_stairs = true;
          stop();
        }
      if(this->descend_stairs)
        {
          ROS_INFO("DESCEND STAIRS DETECTED %lf", double(rng->range));
        }

      this->peak_list.push_back(double(rng->range));
      double avg = 0;
      std::list<double>::iterator it;
      for(it = this->peak_list.begin(); it != this->peak_list.end(); it++) 
      {
        avg += *it;
      }
      avg /= this->peak_list.size();
      this->floor_distance_peak_mean = avg;
      if(this->peak_list.size() > 50){
        this->peak_list.pop_front();
      }
      //ROS_INFO("AVG %lf", this->floor_distance_peak_mean);  
    }
  }

  void setClimbFlippers()
  {
    updateCurrentFlipperAngle(flipper.fr, 0);
    updateCurrentFlipperAngle(flipper.fl, 0);
    publish_flipper();
    ros::Duration(2).sleep();
    updateCurrentFlipperAngle(flipper.rr, 0);
    updateCurrentFlipperAngle(flipper.rl, 0);
    publish_flipper();
  }

  void resetVariables()
  {
    this->isClimbing = true;
    this->floor_distance_peak = 0.0;
    this->floor_distance_peak_count = 0;
    this->peak_list = {};
    this->floor_distance_peak_marker = 0;
    this->floor_distance_peak_mean = 0;
  }

  void startClimbing(int vel)
  {
    geometry_msgs::TwistStamped vel_ts;
    vel_ts.header.frame_id = framename_base_link;
    vel_ts.header.stamp = ros::Time::now();
    vel_ts.twist.linear.x  = vel * velocity.speed.factor;
    vel_ts.twist.linear.y  = vel_ts.twist.linear.z  = 0;
    vel_ts.twist.angular.x = vel_ts.twist.angular.y = vel_ts.twist.angular.z = 0;
    velocity_publisher.publish(vel_ts);
  }

  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    // Publish about velocity
    if(!this->isClimbing && !this->descend_stairs)
    {
      geometry_msgs::TwistStamped vel_ts;
      vel_ts.header.frame_id = framename_base_link;
      vel_ts.header.stamp = ros::Time::now();
      vel_ts.twist.linear.x  = getAxis(joy, velocity.speed);
      vel_ts.twist.linear.y  = vel_ts.twist.linear.z  = 0;
      vel_ts.twist.angular.x = vel_ts.twist.angular.y = 0;
      vel_ts.twist.angular.z = getAxis(joy, velocity.turn)
                                              *((vel_ts.twist.linear.x<0)?-1:1);
      velocity_publisher.publish(vel_ts);
      }
    // Publish about flippers
    
    if(check_climb_start(joy))
    {
      resetVariables();
      setClimbFlippers();
      startClimbing(1);
    }
    else if(!this->isClimbing && !this->descend_stairs)
    {
      updateCurrentFlipperAngle(flipper.fr, joy);
      updateCurrentFlipperAngle(flipper.fl, joy);
      updateCurrentFlipperAngle(flipper.rr, joy);
      updateCurrentFlipperAngle(flipper.rl, joy);
      publish_flipper();
    }
    
  }

  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if(axis.axis < 0 || axis.axis >= joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.name << " is out of range, joy has " << joy->axes.size() << " axes");
      return 0;
    }
    double output = joy->axes[axis.axis] * axis.factor + axis.offset;
    // TODO keep or remove deadzone? may not be needed
    // if(std::abs(output) < axis.max_ * 0.2)
    // {
    //   output = 0.0;
    // }
    return output;
  }

  bool getUpButton(const sensor_msgs::JoyConstPtr &joy, const Flipper &flpr)
  {
    if(flpr.upButton < 0 || flpr.upButton >= joy->buttons.size())
    {
      ROS_ERROR_STREAM("upButton of " << flpr.name << " is out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }
    return joy->buttons[flpr.upButton] > 0;
  }

  bool getDownButton(const sensor_msgs::JoyConstPtr &joy, const Flipper &flpr)
  {
    if(flpr.downButton < 0 || flpr.downButton >= joy->buttons.size())
    {
      ROS_ERROR_STREAM("downButton of " << flpr.name << " is out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }
    return joy->buttons[flpr.downButton] > 0;
  }

  void stop()
  {
    if(velocity_publisher.getNumSubscribers() > 0)
    {
      velocity_publisher.publish(geometry_msgs::TwistStamped());
    }

    if(flipper_publisher.getNumSubscribers() > 0)
    {
      flipper_publisher.publish(geometry_msgs::TwistStamped());
    }
  }
};

} // namespace teleop_jaguar

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_jaguar");

  teleop_jaguar::Teleop teleop;
  teleop.Usage();
  teleop.publish_flipper();
  ros::spin();

  return 0;
}
