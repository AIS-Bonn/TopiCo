//------------------------------------------------------------------------
// File:       main.cpp
// Version:    0.1
// Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
// Package:    opt_control (https://github.com/AIS-Bonn/opt_control)
// License:    BSD
//------------------------------------------------------------------------

// Software License Agreement (BSD License)
// Copyright (c) 2018, Computer Science Institute VI, University of Bonn
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
//------------------------------------------------------------------------

#include "rt_nonfinite.h"
#include "opt_control_lib.h"
#include "opt_control_lib_terminate.h"
#include "opt_control_lib_emxAPI.h"
#include "opt_control_lib_initialize.h"
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <opt_control/OptControlConfig.h>
#include <visualization_msgs/Marker.h>
#include "opt_control/Waypoints.h"

#include "ros/ros.h"


/* Function Declarations */
static emxArray_boolean_T *argInit_1xUnbounded_boolean_T();
static emxArray_real_T *argInit_Unboundedx1_real_T();
static emxArray_real_T *argInit_Unboundedx3_real_T();
static emxArray_int16_T *c_argInit_Unboundedx2xUnbounded();
static emxArray_real_T *c_argInit_Unboundedx5xUnbounded();
static emxArray_boolean_T *c_argInit_UnboundedxUnbounded_b();
static emxArray_real_T *c_argInit_UnboundedxUnbounded_r();
static void main_opt_control_lib();

#define num_axes 3
#define num_traj 1

double P_init[num_axes] = {5};
double V_init[num_axes];
double A_init[num_axes];

double P_wayp[num_axes];
double V_wayp[num_axes];
double A_wayp[num_axes];

double V_max;
double V_min;
double A_max;
double A_min;
double J_max;
double J_min;

double A_global[num_axes];

bool b_sync_V;
bool b_sync_A;
bool b_sync_J;
bool b_sync_W;

bool b_comp_global;
bool b_rotate;
bool b_best_solution;
bool b_hard_vel_limit;
bool b_catch_up;



/* Function Definitions */
static emxArray_boolean_T *argInit_1xUnbounded_boolean_T()
{
  emxArray_boolean_T *result;
  static int iv182[2] = { 1, num_traj };
  int idx1;
  result = emxCreateND_boolean_T(2, iv182);
  for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
    result->data[result->size[0] * idx1] = false;
  }
  return result;
}

static emxArray_real_T *argInit_Unboundedx1_real_T()
{
  emxArray_real_T *result;
  static int iv180[1] = { num_axes };
  int idx0;
  result = emxCreateND_real_T(1, iv180);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    result->data[idx0] = 0.0;
  }
  return result;
}

static emxArray_real_T *argInit_Unboundedx3_real_T()
{
  emxArray_real_T *result;
  static int iv177[2] = { num_axes, 3 };
  int idx0;
  int idx1;
  result = emxCreateND_real_T(2, iv177);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < 3; idx1++) {
      result->data[idx0 + result->size[0] * idx1] = 0.0;
    }
  }
  return result;
}

static emxArray_int16_T *c_argInit_Unboundedx2xUnbounded()
{
  emxArray_int16_T *result;
  static int iv183[3] = { num_axes, 2, num_traj };
  int idx0;
  int idx1;
  int idx2;
  result = emxCreateND_int16_T(3, iv183);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < result->size[2U]; idx2++) {
        result->data[(idx0 + result->size[0] * idx1) + result->size[0] * result->size[1] * idx2] = 0;
      }
    }
  }
  return result;
}

static emxArray_real_T *c_argInit_Unboundedx5xUnbounded()
{
  emxArray_real_T *result;
  static int iv178[3] = { num_axes, 5, num_traj };
  int idx0;
  int idx1;
  int idx2;
  result = emxCreateND_real_T(3, iv178);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < 5; idx1++) {
      for (idx2 = 0; idx2 < result->size[2U]; idx2++) {
        result->data[(idx0 + result->size[0] * idx1) + result->size[0] * result->size[1] * idx2] = 0.0;
      }
    }
  }
  return result;
}

static emxArray_boolean_T *c_argInit_UnboundedxUnbounded_b()
{
  emxArray_boolean_T *result;
  static int iv181[2] = { num_axes, num_traj };
  int idx0;
  int idx1;
  result = emxCreateND_boolean_T(2, iv181);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      result->data[idx0 + result->size[0] * idx1] = false;
    }
  }
  return result;
}

static emxArray_real_T *c_argInit_UnboundedxUnbounded_r()
{
  emxArray_real_T *result;
  static int iv179[2] = { num_axes, num_traj };
  int idx0;
  int idx1;
  result = emxCreateND_real_T(2, iv179);
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      result->data[idx0 + result->size[0] * idx1] = 0.0;
    }
  }
  return result;
}


void waypoints_callback(const opt_control::Waypoints &msg)
{
  for (int i = 0; i < num_axes; i++) {
    P_wayp[i] = msg.P[i];
    V_wayp[i] = msg.V[i];
    A_wayp[i] = msg.A[i];
  }
}


void dynamic_reconfigure_callback(opt_control::OptControlConfig &config, uint32_t level)
{

  V_max             = config.V_max;
  V_min             = config.V_min;
  A_max             = config.A_max;
  A_min             = config.A_min;
  J_max             = config.J_max;
  J_min             = config.J_min;

  b_sync_V          = config.b_sync_V;
  b_sync_A          = config.b_sync_A;
  b_sync_J          = config.b_sync_J;
  b_sync_W          = config.b_sync_W;

  b_rotate          = config.b_rotate;
  b_best_solution   = config.b_best_solution;
  b_hard_vel_limit  = config.b_hard_vel_limit;
  b_catch_up        = config.b_catch_up;

  ROS_INFO("Setting new parameters");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "opt_control_node");

  opt_control_lib_initialize();

  dynamic_reconfigure::Server<opt_control::OptControlConfig> server;
  dynamic_reconfigure::Server<opt_control::OptControlConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  ros::Publisher path_pub                         = nh.advertise<nav_msgs::Path>("trajectory_rollout", 0);
  ros::Publisher marker_waypoint_position_pub     = nh.advertise<visualization_msgs::Marker>( "waypoint_position", 0 );
  ros::Publisher marker_waypoint_velocity_pub     = nh.advertise<visualization_msgs::Marker>( "waypoint_velocity", 0 );
  ros::Publisher marker_waypoint_acceleration_pub = nh.advertise<visualization_msgs::Marker>( "waypoint_acceleration", 0 );
  
  ros::Subscriber waypoint_sub = nh.subscribe("Waypoints", 1, waypoints_callback,ros::TransportHints().tcpNoDelay());

  double markersize = 0.1;
  visualization_msgs::Marker marker_waypoint_position;
  marker_waypoint_position.header.frame_id = "map";
  marker_waypoint_position.type = visualization_msgs::Marker::SPHERE;
  marker_waypoint_position.action = visualization_msgs::Marker::ADD;
  marker_waypoint_position.color.r = 1.0;
  marker_waypoint_position.color.g = 0.0;
  marker_waypoint_position.color.b = 0.0;
  marker_waypoint_position.color.a = 0.5;
  marker_waypoint_position.scale.x = markersize;
  marker_waypoint_position.scale.y = markersize;
  marker_waypoint_position.scale.z = markersize;

  visualization_msgs::Marker marker_waypoint_velocity;
  marker_waypoint_velocity.points.resize(2);
  marker_waypoint_velocity.header.frame_id = "map";
  marker_waypoint_velocity.type = visualization_msgs::Marker::ARROW;
  marker_waypoint_velocity.action = visualization_msgs::Marker::ADD;
  marker_waypoint_velocity.color.r = 0.0;
  marker_waypoint_velocity.color.g = 1.0;
  marker_waypoint_velocity.color.b = 0.0;
  marker_waypoint_velocity.color.a = 0.5;
  marker_waypoint_velocity.scale.x = markersize * 0.5;
  marker_waypoint_velocity.scale.y = markersize;
  marker_waypoint_velocity.scale.z = 0.0;

  visualization_msgs::Marker marker_waypoint_acceleration;
  marker_waypoint_acceleration.points.resize(2);
  marker_waypoint_acceleration.header.frame_id = "map";
  marker_waypoint_acceleration.type = visualization_msgs::Marker::ARROW;
  marker_waypoint_acceleration.action = visualization_msgs::Marker::ADD;
  marker_waypoint_acceleration.color.r = 0.0;
  marker_waypoint_acceleration.color.g = 0.0;
  marker_waypoint_acceleration.color.b = 1.0;
  marker_waypoint_acceleration.color.a = 0.5;
  marker_waypoint_acceleration.scale.x = markersize * 0.5;
  marker_waypoint_acceleration.scale.y = markersize;
  marker_waypoint_acceleration.scale.z = 0.0;

  nav_msgs::Path path_rollout;
  path_rollout.header.frame_id = "map";

  //Inputs
  emxArray_real_T *State_start_in;
  emxArray_real_T *Waypoints_in;
  emxArray_real_T *V_max_in;
  emxArray_real_T *V_min_in;
  emxArray_real_T *A_max_in;
  emxArray_real_T *A_min_in;
  emxArray_real_T *J_max_in;
  emxArray_real_T *J_min_in;
  emxArray_real_T *A_global_in;
  emxArray_boolean_T *b_sync_V_in;
  emxArray_boolean_T *b_sync_A_in;
  emxArray_boolean_T *b_sync_J_in;
  emxArray_boolean_T *b_sync_W_in;
  emxArray_boolean_T *b_rotate_in;
  emxArray_boolean_T *b_best_solution_in;
  emxArray_boolean_T *b_hard_vel_limit_in;
  emxArray_boolean_T *b_catch_up_in;
  bool b_comp_global_in;
  emxArray_int16_T *solution_in;

  //Outputs
  emxArray_struct0_T *J_setp_struct;
  emxArray_int16_T *solution_out;
  emxArray_real_T *T_waypoints;
  emxArray_real_T *P_rollout;
  emxArray_real_T *V_rollout;
  emxArray_real_T *A_rollout;
  emxArray_real_T *J_rollout;
  emxArray_real_T *t_rollout;
  emxInitArray_struct0_T(&J_setp_struct, 2);
  emxInitArray_int16_T(&solution_out, 3);
  emxInitArray_real_T(&T_waypoints, 2);
  emxInitArray_real_T(&P_rollout, 2);
  emxInitArray_real_T(&V_rollout, 2);
  emxInitArray_real_T(&A_rollout, 2);
  emxInitArray_real_T(&J_rollout, 2);
  emxInitArray_real_T(&t_rollout, 2);


  State_start_in = argInit_Unboundedx3_real_T();
  Waypoints_in = c_argInit_Unboundedx5xUnbounded();
  V_max_in = c_argInit_UnboundedxUnbounded_r();
  V_min_in = c_argInit_UnboundedxUnbounded_r();
  A_max_in = c_argInit_UnboundedxUnbounded_r();
  A_min_in = c_argInit_UnboundedxUnbounded_r();
  J_max_in = c_argInit_UnboundedxUnbounded_r();
  J_min_in = c_argInit_UnboundedxUnbounded_r();
  A_global_in = argInit_Unboundedx1_real_T();
  b_sync_V_in = c_argInit_UnboundedxUnbounded_b();
  b_sync_A_in = c_argInit_UnboundedxUnbounded_b();
  b_sync_J_in = c_argInit_UnboundedxUnbounded_b();
  b_sync_W_in = c_argInit_UnboundedxUnbounded_b();
  b_rotate_in = argInit_1xUnbounded_boolean_T();
  b_best_solution_in = c_argInit_UnboundedxUnbounded_b();
  b_hard_vel_limit_in = c_argInit_UnboundedxUnbounded_b();
  b_catch_up_in = c_argInit_UnboundedxUnbounded_b();
  solution_in = c_argInit_Unboundedx2xUnbounded();

  while (ros::ok()){

    ros::spinOnce();
    double t = ros::Time::now().toSec();
    ros::Time t_header(t);

    marker_waypoint_position.header.stamp = t_header;
    marker_waypoint_position.pose.position.x = P_wayp[0];
    marker_waypoint_position.pose.position.y = P_wayp[1];
    marker_waypoint_position.pose.position.z = P_wayp[2];
    marker_waypoint_position_pub.publish(marker_waypoint_position);

    marker_waypoint_velocity.header.stamp = t_header;
    marker_waypoint_velocity.points[0].x = P_wayp[0];
    marker_waypoint_velocity.points[0].y = P_wayp[1];
    marker_waypoint_velocity.points[0].z = P_wayp[2];
    marker_waypoint_velocity.points[1].x = P_wayp[0] + V_wayp[0];
    marker_waypoint_velocity.points[1].y = P_wayp[1] + V_wayp[1];
    marker_waypoint_velocity.points[1].z = P_wayp[2] + V_wayp[2];
    marker_waypoint_velocity_pub.publish(marker_waypoint_velocity);

    marker_waypoint_acceleration.header.stamp = t_header;
    marker_waypoint_acceleration.points[0].x = P_wayp[0];
    marker_waypoint_acceleration.points[0].y = P_wayp[1];
    marker_waypoint_acceleration.points[0].z = P_wayp[2];
    marker_waypoint_acceleration.points[1].x = P_wayp[0] + A_wayp[0];
    marker_waypoint_acceleration.points[1].y = P_wayp[1] + A_wayp[1];
    marker_waypoint_acceleration.points[1].z = P_wayp[2] + A_wayp[2];
    marker_waypoint_acceleration_pub.publish(marker_waypoint_acceleration);


    for (int idx_traj = 0; idx_traj < num_traj; idx_traj++) {
      for (int idx_axis = 0; idx_axis < num_axes; idx_axis++) {
        State_start_in->data[idx_axis + State_start_in->size[0] * 0] = P_init[idx_axis];
        State_start_in->data[idx_axis + State_start_in->size[0] * 1] = V_init[idx_axis];
        State_start_in->data[idx_axis + State_start_in->size[0] * 2] = A_init[idx_axis];
        Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 0) + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj] = P_wayp[idx_axis];
        Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 1) + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj] = V_wayp[idx_axis];
        Waypoints_in->data[(idx_axis + Waypoints_in->size[0] * 2) + Waypoints_in->size[0] * Waypoints_in->size[1] * idx_traj] = A_wayp[idx_axis];
        V_max_in->data[idx_axis + V_max_in->size[0] * idx_traj] = V_max;
        V_min_in->data[idx_axis + V_min_in->size[0] * idx_traj] = V_min;
        A_max_in->data[idx_axis + A_max_in->size[0] * idx_traj] = A_max;
        A_min_in->data[idx_axis + A_min_in->size[0] * idx_traj] = A_min;
        J_max_in->data[idx_axis + J_max_in->size[0] * idx_traj] = J_max;
        J_min_in->data[idx_axis + J_min_in->size[0] * idx_traj] = J_min;
        A_global_in->data[idx_axis]  = A_global[idx_axis];
        b_comp_global_in             = b_comp_global;
        b_sync_V_in->data[idx_axis + b_sync_V_in->size[0] * idx_traj] = b_sync_V;
        b_sync_A_in->data[idx_axis + b_sync_A_in->size[0] * idx_traj] = b_sync_A;
        b_sync_J_in->data[idx_axis + b_sync_J_in->size[0] * idx_traj] = b_sync_J;
        b_sync_W_in->data[idx_axis + b_sync_W_in->size[0] * idx_traj] = b_sync_W;
        b_rotate_in->data[b_rotate_in->size[0] * idx_axis]                            = b_rotate;
        b_best_solution_in->data[idx_axis + b_best_solution_in->size[0] * idx_traj]   = b_best_solution;
        b_hard_vel_limit_in->data[idx_axis + b_hard_vel_limit_in->size[0] * idx_traj] = b_hard_vel_limit;
        b_catch_up_in->data[idx_axis + b_catch_up_in->size[0] * idx_traj]             = b_catch_up;
      }
    }

    double ts_rollout = 0.1;

    double t_timing_start = clock();
    opt_control_lib(State_start_in, Waypoints_in, V_max_in, V_min_in, A_max_in, A_min_in, J_max_in, J_min_in, A_global_in, b_comp_global_in, b_sync_V_in, b_sync_A_in, b_sync_J_in, b_sync_W_in, b_rotate_in, b_best_solution_in, b_hard_vel_limit_in, b_catch_up_in, solution_in, ts_rollout, J_setp_struct, solution_out, T_waypoints, P_rollout, V_rollout, A_rollout, J_rollout, t_rollout);
    double t_timing_stop = clock();

    ROS_INFO("Elapsed time is %.2fms",(t_timing_stop - t_timing_start)/CLOCKS_PER_SEC*1000);
  
    

    int rollout_iterations = P_rollout->size[1];
    path_rollout.header.stamp = t_header;
    path_rollout.poses.resize(rollout_iterations);
    for (int i = 0; i < rollout_iterations; i++) {
      path_rollout.poses[i].pose.position.x = P_rollout->data[i * P_rollout->size[0] + 0];
      path_rollout.poses[i].pose.position.y = P_rollout->data[i * P_rollout->size[0] + 1];
      path_rollout.poses[i].pose.position.z = P_rollout->data[i * P_rollout->size[0] + 2];
    }
    path_pub.publish(path_rollout);

    loop_rate.sleep();

  }

  emxDestroyArray_real_T(J_rollout);
  emxDestroyArray_real_T(A_rollout);
  emxDestroyArray_real_T(V_rollout);
  emxDestroyArray_real_T(P_rollout);
  emxDestroyArray_real_T(t_rollout);
  emxDestroyArray_real_T(T_waypoints);
  emxDestroyArray_int16_T(solution_out);
  emxDestroyArray_struct0_T(J_setp_struct);
  emxDestroyArray_int16_T(solution_in);
  emxDestroyArray_boolean_T(b_catch_up_in);
  emxDestroyArray_boolean_T(b_hard_vel_limit_in);
  emxDestroyArray_boolean_T(b_best_solution_in);
  emxDestroyArray_boolean_T(b_rotate_in);
  emxDestroyArray_boolean_T(b_sync_W_in);
  emxDestroyArray_boolean_T(b_sync_J_in);
  emxDestroyArray_boolean_T(b_sync_A_in);
  emxDestroyArray_boolean_T(b_sync_V_in);
  emxDestroyArray_real_T(A_global_in);
  emxDestroyArray_real_T(J_min_in);
  emxDestroyArray_real_T(J_max_in);
  emxDestroyArray_real_T(A_min_in);
  emxDestroyArray_real_T(A_max_in);
  emxDestroyArray_real_T(V_min_in);
  emxDestroyArray_real_T(V_max_in);
  emxDestroyArray_real_T(Waypoints_in);
  emxDestroyArray_real_T(State_start_in);

  opt_control_lib_terminate();

  return 0;
}