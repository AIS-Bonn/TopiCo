// ---------------------------------------------------------------------
// Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
// Version:    2021-03-30 12:12:52
// Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
// License:    BSD
// ---------------------------------------------------------------------

// Software License Agreement (BSD License)
// Copyright (c) 2021, Computer Science Institute VI, University of Bonn
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of University of Bonn, Computer Science Institute
//   VI nor the names of its contributors may be used to endorse or
//   promote products derived from this software without specific
//   prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <topico/TopiCoConfig.h>

#include "rt_nonfinite.h"
#include "topico_wrapper.h"
#include "topico_wrapper_terminate.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <sstream>
#include <stdexcept>
#include <string>


bool b_wayp_updated = false;
bool b_init_updated = false;
uint num_dim  = 3;
uint num_wayp = 1;

// Declare Inputs
coder::array<double, 2U> State_start;
coder::array<double, 3U> Waypoints;
coder::array<double, 2U> V_max;
coder::array<double, 2U> V_min;
coder::array<double, 2U> A_max;
coder::array<double, 2U> A_min;
coder::array<double, 2U> J_max;
coder::array<double, 2U> J_min;
coder::array<double, 1U> A_global;
coder::array<bool, 2U> b_sync_V;
coder::array<bool, 2U> b_sync_A;
coder::array<bool, 2U> b_sync_J;
coder::array<bool, 2U> b_sync_W;
coder::array<bool, 2U> b_rotate;
coder::array<bool, 2U> b_hard_V_lim;
coder::array<bool, 2U> b_catch_up;
coder::array<signed char, 2U> direction;
double ts_rollout;

void init_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  b_init_updated = true;
  State_start[(0 + num_dim * 0)] = msg->pose.pose.position.x; //Initial X Position
  State_start[(1 + num_dim * 0)] = msg->pose.pose.position.y; //Initial Y Position
  State_start[(2 + num_dim * 0)] = msg->pose.pose.position.z; //Initial Z Position
  State_start[(0 + num_dim * 1)] = msg->twist.twist.linear.x; //Initial X Velocity
  State_start[(1 + num_dim * 1)] = msg->twist.twist.linear.y; //Initial Y Velocity
  State_start[(2 + num_dim * 1)] = msg->twist.twist.linear.z; //Initial Z Velocity
  State_start[(0 + num_dim * 2)] = 0.0;                       //Initial X Acceleration
  State_start[(1 + num_dim * 2)] = 0.0;                       //Initial Y Acceleration
  State_start[(2 + num_dim * 2)] = 0.0;                       //Initial Z Acceleration
}

void wayp_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  b_wayp_updated = true;
  int idx_wayp = 0;
  Waypoints[(0 + num_dim * 0) + num_dim * 5 * idx_wayp] = msg->pose.pose.position.x; //Waypoint X Position
  Waypoints[(1 + num_dim * 0) + num_dim * 5 * idx_wayp] = msg->pose.pose.position.y; //Waypoint Y Position
  Waypoints[(2 + num_dim * 0) + num_dim * 5 * idx_wayp] = msg->pose.pose.position.z; //Waypoint Z Position
  Waypoints[(0 + num_dim * 1) + num_dim * 5 * idx_wayp] = msg->twist.twist.linear.x; //Waypoint X Velocity
  Waypoints[(1 + num_dim * 1) + num_dim * 5 * idx_wayp] = msg->twist.twist.linear.y; //Waypoint Y Velocity
  Waypoints[(2 + num_dim * 1) + num_dim * 5 * idx_wayp] = msg->twist.twist.linear.z; //Waypoint Z Velocity
  Waypoints[(0 + num_dim * 2) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint X Acceleration
  Waypoints[(1 + num_dim * 2) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint Y Acceleration
  Waypoints[(2 + num_dim * 2) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint Z Acceleration
  Waypoints[(0 + num_dim * 3) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint X Movement Velocity
  Waypoints[(1 + num_dim * 3) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint Y Movement Velocity
  Waypoints[(2 + num_dim * 3) + num_dim * 5 * idx_wayp] = 0.0;                       //Waypoint Z Movement Velocity
  Waypoints[(0 + num_dim * 4) + num_dim * 5 * idx_wayp] = 0.0;                       //reserved
  Waypoints[(1 + num_dim * 4) + num_dim * 5 * idx_wayp] = 0.0;                       //reserved
  Waypoints[(2 + num_dim * 4) + num_dim * 5 * idx_wayp] = 0.0;                       //reserved
}


void dynamic_reconfigure_callback(topico::TopiCoConfig &config, uint32_t level)
{
  for (int idx_dim = 0; idx_dim < num_dim; idx_dim++) {  
    A_global[idx_dim] = 0.0; 
    for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
      V_max[idx_dim + num_dim * idx_wayp]        = config.V_max;
      V_min[idx_dim + num_dim * idx_wayp]        = config.V_min;
      A_max[idx_dim + num_dim * idx_wayp]        = config.A_max;
      A_min[idx_dim + num_dim * idx_wayp]        = config.A_min;
      J_max[idx_dim + num_dim * idx_wayp]        = config.J_max;
      J_min[idx_dim + num_dim * idx_wayp]        = config.J_min;
      b_sync_V[idx_dim + num_dim * idx_wayp]     = config.b_sync_V;
      b_sync_A[idx_dim + num_dim * idx_wayp]     = config.b_sync_A;
      b_sync_J[idx_dim + num_dim * idx_wayp]     = config.b_sync_J;
      b_sync_W[idx_dim + num_dim * idx_wayp]     = config.b_sync_W;
      b_hard_V_lim[idx_dim + num_dim * idx_wayp] = config.b_hard_V_lim;
      b_catch_up[idx_dim + num_dim * idx_wayp]   = config.b_catch_up;
      direction[idx_dim + num_dim * idx_wayp]    = config.direction;
    }
  }  
  for (int idx_dim = 0; idx_dim < num_dim-1; idx_dim++) {  
    for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
      b_rotate[idx_dim + num_dim * idx_wayp]     = config.b_rotate;
    }
  }
  ts_rollout = config.ts_rollout;
  
  printf("Debug: Setting new parameters!\n");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topico");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(100);
  
  ros::Subscriber wayp_sub = nh.subscribe("wayp_odometry", 1, wayp_callback,ros::TransportHints().tcpNoDelay());
  ros::Subscriber init_sub = nh.subscribe("init_odometry", 1, init_callback,ros::TransportHints().tcpNoDelay());
  ros::Publisher path_pub  = nh.advertise<nav_msgs::Path>("trajectory_rollout", 0);
  
  std::string map_frame;
  nh.param<std::string>( "frame_id", map_frame, "world" );

  // Declare Outputs
  coder::array<struct0_T, 2U> J_setp_struct;
  coder::array<int, 2U> solution_out;
  coder::array<double, 2U> T_waypoints;
  coder::array<double, 2U> P;
  coder::array<double, 2U> V;
  coder::array<double, 2U> A;
  coder::array<double, 2U> J;
  coder::array<double, 2U> t;

  // Resize Inputs
  State_start.set_size(num_dim,3);
  Waypoints.set_size(num_dim,5,num_wayp);
  V_max.set_size(num_dim,num_wayp);
  V_min.set_size(num_dim,num_wayp);
  A_max.set_size(num_dim,num_wayp);
  A_min.set_size(num_dim,num_wayp);
  J_max.set_size(num_dim,num_wayp);
  J_min.set_size(num_dim,num_wayp);
  A_global.set_size(num_dim);
  b_sync_V.set_size(num_dim,num_wayp);
  b_sync_A.set_size(num_dim,num_wayp);
  b_sync_J.set_size(num_dim,num_wayp);
  b_sync_W.set_size(num_dim,num_wayp);
  b_rotate.set_size(num_dim-1,num_wayp);
  b_hard_V_lim.set_size(num_dim,num_wayp);
  b_catch_up.set_size(num_dim,num_wayp);
  direction.set_size(num_dim,num_wayp);
  
  bool b_initialized = false;
  
  dynamic_reconfigure::Server<topico::TopiCoConfig> server;
  dynamic_reconfigure::Server<topico::TopiCoConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);
 
  nav_msgs::Path path_rollout;
  
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time t_now = ros::Time::now();
    
    if (b_wayp_updated && b_init_updated) {
      b_initialized = true;
    }

    if (b_initialized && (b_init_updated || b_wayp_updated)) { // only replan when new data arrived...
      b_wayp_updated = false;
      b_init_updated = false;
 
      topico_wrapper(State_start, Waypoints, V_max, V_min, A_max, A_min, J_max, J_min, A_global, b_sync_V, b_sync_A, b_sync_J, b_sync_W, b_rotate, b_hard_V_lim, b_catch_up, direction, ts_rollout, J_setp_struct,solution_out, T_waypoints, P, V, A, J, t);

      int size_rollout = P.size(1);
      path_rollout.poses.resize(size_rollout);
      path_rollout.header.stamp = t_now;
      path_rollout.header.frame_id = map_frame;
      for (int idx = 0; idx < size_rollout; idx++)
      {
        path_rollout.poses[idx].pose.position.x = P[3*idx+0];
        path_rollout.poses[idx].pose.position.y = P[3*idx+1];
        path_rollout.poses[idx].pose.position.z = P[3*idx+2];
      }
      path_pub.publish(path_rollout);
    } else if(!b_initialized) {
      printf("Warning: Initial state and/or waypoint not published yet!\n");       
    }
    loop_rate.sleep();
  }

  topico_wrapper_terminate();
  return 0;
}

