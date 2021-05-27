% ---------------------------------------------------------------------
% Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
% Version:    2021-03-18 12:09:55
% Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
% License:    BSD
% ---------------------------------------------------------------------

% Software License Agreement (BSD License)
% Copyright (c) 2021, Computer Science Institute VI, University of Bonn
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
%% --------------------------------------------------------------------

clearvars;
close all;
addpath(genpath(pwd));

%% ----------   Variables    ----------
index_example = 1;

switch index_example
    case 1
        State_start       = [ 0.0,  0.0,  0.0];
        Waypoints(:,:,1)  = [ 2.0,  0.0,  0.0,  0.0,  0.0];
        Waypoints(:,:,2)  = [ 0.0,  0.0,  0.0,  0.0,  0.0];
        V_max             = [ 1.0,  1.0];
        V_min             = [-1.0, -1.0];
        A_max             = [ 0.5,  0.5];
        A_min             = [-0.5, -0.5];
        J_max             = [ 1.0,  1.0];
        J_min             = [-1.0, -1.0];
        A_global          =   0.0;
    case 2
        State_start       = [ 0.0,  0.0,  0.0];
        Waypoints(:,:,1)  = [ 3.0,  0.5,  0.0,  0.0,  0.0];
        Waypoints(:,:,2)  = [ 4.0,  0.0,  0.0,  0.0,  0.0];
        V_max             = [ 1.0,  0.2];
        V_min             = [-1.0, -1.0];
        A_max             = [ 0.5,  0.5];
        A_min             = [-0.5, -0.5];
        J_max             = [ 1.0,  1.0];
        J_min             = [-1.0, -1.0];
        A_global          =   0.0;
     case 3
        State_start       = [ 0.0,  0.0,  0.0];
        Waypoints(:,:,1)  = [ 2.0,  0.0,  0.0,  0.0,  0.0];
        Waypoints(:,:,2)  = [ 0.0,  0.0,  0.0,  0.0,  0.0];
        V_max             = [ 1.0,  1.0];
        V_min             = [-1.0, -1.0];
        A_max             = [ 0.5,  0.5];
        A_min             = [-0.5, -0.5];
        J_max             = [ Inf,  Inf];
        J_min             = [-Inf, -Inf];
        A_global          =   0.0;
    case 4
        State_start       = [ 0.0,  0.5,  0.0;
                              0.0,  0.5,  0.0;
                              0.0,  0.5,  0.0];
        Waypoints(:,:,1)  = [ 0.0,  0.5,  0.0,  0.0,  0.0;
                              3.0,  0.5,  0.0,  0.0,  0.0;
                              0.0,  0.5,  0.0,  0.0,  0.0];
        Waypoints(:,:,2)  = [ 3.0,  0.5,  0.0,  0.0,  0.0;
                              3.0,  0.5,  0.0,  0.0,  0.0;
                              0.0, -0.5,  0.0,  0.0,  0.0];
        V_max             = [ 1.0,  1.0;
                              1.0,  1.0;
                              1.0,  1.0];
        V_min             = [-1.0, -1.0;
                             -1.0, -1.0;
                             -1.0, -1.0];
        A_max             = [ 0.5,  0.5;
                              0.5,  0.5;
                              0.5,  0.5];
        A_min             = [-0.5, -0.5;
                             -0.5, -0.5;
                             -0.5, -0.5];
        J_max             = [ 1.0,  1.0;
                              1.0,  1.0;
                              1.0,  1.0];
        J_min             = [-1.0, -1.0;
                             -1.0, -1.0;
                             -1.0, -1.0];
        A_global          = [ 0.0;
                              0.0;
                              0.0];
     case 5
        State_start       = [ 0.0,  0.0,  0.0];
        Waypoints(:,:,1)  = [ 2.0,  NaN,  NaN,  0.0,  0.0];
        Waypoints(:,:,2)  = [ 0.0,  0.0,  0.0,  0.0,  0.0];
        V_max             = [ 1.0,  1.0];
        V_min             = [-1.0, -1.0];
        A_max             = [ 0.5,  0.5];
        A_min             = [-0.5, -0.5];
        J_max             = [ 1.0,  1.0];
        J_min             = [-1.0, -1.0];
        A_global          =   0.0;
    case 6
        rng(6);
        num_axes          = randi(7);
        num_waypoints     = randi(5);

        State_start       =  0.0*rand(num_axes,3);
        
        V_max             =  1.0*ones(num_axes,num_waypoints);
        V_min             = -1.0*ones(num_axes,num_waypoints);
        A_max             =  0.5*ones(num_axes,num_waypoints);
        A_min             = -0.5*ones(num_axes,num_waypoints);
        J_max             =  1.0*ones(num_axes,num_waypoints);
        J_min             = -1.0*ones(num_axes,num_waypoints);
        A_global          =  0.0*ones(num_axes,1);
        
        P_waypoint_max    =  5.0.*ones(num_axes,num_waypoints);
        P_waypoint_min    = -5.0.*ones(num_axes,num_waypoints);
        V_waypoint_max    =  0.4.*ones(num_axes,num_waypoints);
        V_waypoint_min    = -0.4.*ones(num_axes,num_waypoints);
        A_waypoint_max    =  0.3.*ones(num_axes,num_waypoints);
        A_waypoint_min    = -0.3.*ones(num_axes,num_waypoints);
        PV_waypoint_max   =  0.2.*ones(num_axes,num_waypoints);
        PV_waypoint_min   = -0.2.*ones(num_axes,num_waypoints);
        PA_waypoint_max   =  0.0.*ones(num_axes,num_waypoints);
        PA_waypoint_min   =  0.0.*ones(num_axes,num_waypoints);
        
        P_waypoints       = reshape(P_waypoint_min+(P_waypoint_max-P_waypoint_min).*rand(num_axes,num_waypoints),num_axes,1,num_waypoints);
        V_waypoints       = reshape(V_waypoint_min+(V_waypoint_max-V_waypoint_min).*rand(num_axes,num_waypoints),num_axes,1,num_waypoints);
        A_waypoints       = reshape(A_waypoint_min+(A_waypoint_max-A_waypoint_min).*rand(num_axes,num_waypoints),num_axes,1,num_waypoints);
        PV_waypoints      = reshape(PV_waypoint_min+(PV_waypoint_max-PV_waypoint_min).*rand(num_axes,num_waypoints),num_axes,1,num_waypoints);
        PA_waypoints      = reshape(PA_waypoint_min+(PA_waypoint_max-PA_waypoint_min).*rand(num_axes,num_waypoints),num_axes,1,num_waypoints);
        Waypoints         = cat(2,P_waypoints,V_waypoints,A_waypoints,PV_waypoints,PA_waypoints);
    otherwise
        disp('Error: Please select a valid example!');
end

num_axes          = size(Waypoints,1);
num_waypoints     = size(Waypoints,3);
b_sync_V          = true(num_axes,num_waypoints);
b_sync_A          = true(num_axes,num_waypoints);
b_sync_J          = false(num_axes,num_waypoints);
b_sync_W          = false(num_axes,num_waypoints);
b_rotate          = false(num_axes-1,num_waypoints);
b_hard_V_lim      = false(num_axes,num_waypoints);
b_catch_up        = true(num_axes,num_waypoints);
direction         = zeros(num_axes,num_waypoints,'int8');
ts_rollout        = 0.01;


%% ----------   Compute    ----------
tic;
[J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = topico(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_rotate,b_hard_V_lim,b_catch_up,direction,ts_rollout);
%[J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = topico_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_rotate,b_hard_V_lim,b_catch_up,direction,ts_rollout);
t_elapsed = toc;


%% ----------     Output     ----------
disp(['Debug: Generated trajectory for ',int2str(num_axes),' axes and ',int2str(num_waypoints),' waypoints in ',num2str(t_elapsed),'s!']);
show_trajectory_1D;

if (index_example == 4)
    show_trajectory_3D;
end
