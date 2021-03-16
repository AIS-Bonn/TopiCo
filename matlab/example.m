%------------------------------------------------------------------------
% File:       example.m
% Version:    2018-08-29 10:37:04
% Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
% Package:    opt_control (https://github.com/AIS-Bonn/opt_control)
% License:    BSD
%------------------------------------------------------------------------

% Software License Agreement (BSD License)
% Copyright (c) 2018, Computer Science Institute VI, University of Bonn
% All rights reserved.
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
% 
% * Redistributions of source code must retain the above copyright
%   notice, this list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above
%   copyright notice, this list of conditions and the following
%   disclaimer in the documentation and/or other materials provided
%   with the distribution.
% * Neither the name of University of Bonn, Computer Science Institute
%   VI nor the names of its contributors may be used to endorse or
%   promote products derived from this software without specific
%   prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%------------------------------------------------------------------------

clearvars;
%% ----------   Variables    ----------
index_example = 1;

switch index_example
    case 1
        num_axes         = 1;
        num_trajectories = 2;
        State_start      = [ 0.0  0.0  0.0];
        Waypoints(:,:,1) = [ 2.0  0.0  0.0  0.0  0.0];
        Waypoints(:,:,2) = [ 0.0  0.0  0.0  0.0  0.0];
        V_max            = [ 1.0  1.0];
        V_min            = [-1.0 -1.0];
        A_max            = [ 0.5  0.5];
        A_min            = [-0.5 -0.5];
        J_max            = [ 1.0  1.0];
        J_min            = [-1.0 -1.0];
        A_global         =   0.0;
    case 2
        num_axes         = 1;
        num_trajectories = 2;
        State_start      = [ 0.0  0.0  0.0];
        Waypoints(:,:,1) = [ 3.0  0.5  0.0  0.0  0.0];
        Waypoints(:,:,2) = [ 4.0  0.0  0.0  0.0  0.0];
        V_max            = [ 1.0  0.2];
        V_min            = [-1.0 -1.0];
        A_max            = [ 0.5  0.5];
        A_min            = [-0.5 -0.5];
        J_max            = [ 1.0  1.0];
        J_min            = [-1.0 -1.0];
        A_global         =   0.0;
     case 3
        num_axes         = 1;
        num_trajectories = 2;
        State_start      = [ 0.0  0.0  0.0];
        Waypoints(:,:,1) = [ 2.0  0.0  0.0  0.0  0.0];
        Waypoints(:,:,2) = [ 0.0  0.0  0.0  0.0  0.0];
        V_max            = [ 1.0  1.0];
        V_min            = [-1.0 -1.0];
        A_max            = [ 0.5  0.5];
        A_min            = [-0.5 -0.5];
        J_max            = [ Inf  Inf];
        J_min            = [-Inf -Inf];
        A_global         =   0.0;
    case 4
        num_axes         = 2;
        num_trajectories = 2;
        State_start      = [ 0.0  0.0  0.0;
                             0.0  0.0  0.0];
        Waypoints(:,:,1) = [ 1.0  0.0  0.0  0.0  0.0;
                             3.0  0.0  0.0  0.0  0.0];
        Waypoints(:,:,2) = [ 0.0  0.0  0.0  0.0  0.0;
                             0.0  0.0  0.0  0.0  0.0];
        V_max            = [ 1.0  1.0;
                             1.0  1.0];
        V_min            = [-1.0 -1.0;
                            -1.0 -1.0];
        A_max            = [ 0.5  0.5;
                             0.5  0.5];
        A_min            = [-0.5 -0.5;
                            -0.5 -0.5];
        J_max            = [ 1.0  1.0;
                             1.0  1.0];
        J_min            = [-1.0 -1.0;
                            -1.0 -1.0];
        A_global         = [ 0.0;
                             0.0];
     case 5
        num_axes         = 1;
        num_trajectories = 2;
        State_start      = [ 0.0  0.0  0.0];
        Waypoints(:,:,1) = [ 2.0  NaN  NaN  0.0  0.0];
        Waypoints(:,:,2) = [ 0.0  0.0  0.0  0.0  0.0];
        V_max            = [ 1.0  1.0];
        V_min            = [-1.0 -1.0];
        A_max            = [ 0.5  0.5];
        A_min            = [-0.5 -0.5];
        J_max            = [ 1.0  1.0];
        J_min            = [-1.0 -1.0];
        A_global         =   0.0;
    case 6
        rng(6);
        num_axes         = randi(10);
        num_trajectories = randi(5);

        State_start      =  0.0*rand(num_axes,3);
        
        V_max            =  1.0*ones(num_axes,num_trajectories);
        V_min            = -1.0*ones(num_axes,num_trajectories);
        A_max            =  0.5*ones(num_axes,num_trajectories);
        A_min            = -0.5*ones(num_axes,num_trajectories);
        J_max            =  1.0*ones(num_axes,num_trajectories);
        J_min            = -1.0*ones(num_axes,num_trajectories);
        A_global         =  0.0*ones(num_axes,1);
        
        P_waypoint_max   =  5.0.*ones(num_axes,num_trajectories);
        P_waypoint_min   = -5.0.*ones(num_axes,num_trajectories);
        V_waypoint_max   =  0.4.*ones(num_axes,num_trajectories);
        V_waypoint_min   = -0.4.*ones(num_axes,num_trajectories);
        A_waypoint_max   =  0.3.*ones(num_axes,num_trajectories);
        A_waypoint_min   = -0.3.*ones(num_axes,num_trajectories);
        PV_waypoint_max  =  0.2.*ones(num_axes,num_trajectories);
        PV_waypoint_min  = -0.2.*ones(num_axes,num_trajectories);
        PA_waypoint_max  =  0.0.*ones(num_axes,num_trajectories);
        PA_waypoint_min  =  0.0.*ones(num_axes,num_trajectories);
        
        P_waypoints      = reshape(P_waypoint_min+(P_waypoint_max-P_waypoint_min).*rand(num_axes,num_trajectories),num_axes,1,num_trajectories);
        V_waypoints      = reshape(V_waypoint_min+(V_waypoint_max-V_waypoint_min).*rand(num_axes,num_trajectories),num_axes,1,num_trajectories);
        A_waypoints      = reshape(A_waypoint_min+(A_waypoint_max-A_waypoint_min).*rand(num_axes,num_trajectories),num_axes,1,num_trajectories);
        PV_waypoints     = reshape(PV_waypoint_min+(PV_waypoint_max-PV_waypoint_min).*rand(num_axes,num_trajectories),num_axes,1,num_trajectories);
        PA_waypoints     = reshape(PA_waypoint_min+(PA_waypoint_max-PA_waypoint_min).*rand(num_axes,num_trajectories),num_axes,1,num_trajectories);
        Waypoints        = cat(2,P_waypoints,V_waypoints,A_waypoints,PV_waypoints,PA_waypoints);
        
    otherwise
        disp('please select a valid example!');
end

b_comp_global    = false;
b_sync_V         =  true(num_axes,num_trajectories);
b_sync_A         =  true(num_axes,num_trajectories);
b_sync_J         = false(num_axes,num_trajectories);
b_sync_W         =  true(num_axes,num_trajectories);
b_rotate         = false(1,num_trajectories);
b_best_solution  =  true(num_axes,num_trajectories);
b_hard_vel_limit = false(num_axes,num_trajectories);
b_catch_up       =  true(num_axes,num_trajectories);
solution_in      = -1 * ones(num_axes,2,num_trajectories,'int16');
ts_rollout       = 0.01;


%% ----------   Calculate    ----------
tic;
[J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = opt_control_lib_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,solution_in,ts_rollout);
toc;


%% ----------     Output     ----------
disp(['num_axes = ',num2str(num_axes)]);
disp(['num_trajectories = ',num2str(num_trajectories)]);
T_rollout = max(sum(T_waypoints,2));
show_trajectory_1D;





