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

function [J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = topico(State_start,Waypoints,V_max_in,V_min_in,A_max_in,A_min_in,J_max_in,J_min_in,A_global,b_sync_V_in,b_sync_A_in,b_sync_J_in,b_sync_W_in,b_rotate_in,b_hard_V_lim_in,b_catch_up_in,direction_in,ts_rollout) %#codegen
    
    num_axes = size(Waypoints,1);
    num_waypoints = size(Waypoints,3);
    
    t_setp = cell(num_axes,num_waypoints);
    for index_waypoint = 1:num_waypoints %coder
        for index_axis = 1:num_axes
            t_setp{index_axis,index_waypoint} = 0.0;
        end
    end
    
    J_setp = cell(num_axes,num_waypoints);
    for index_waypoint = 1:num_waypoints %coder
        for index_axis = 1:num_axes
            J_setp{index_axis,index_waypoint} = 0.0;
        end
    end
    
    T_waypoints = zeros(num_axes,num_waypoints);
    solution_out = -1 * ones(num_axes,num_waypoints,'int32');
    
    check_inputs(State_start,Waypoints,V_max_in,V_min_in,A_max_in,A_min_in,J_max_in,J_min_in,A_global,b_sync_V_in,b_sync_A_in,b_sync_J_in,b_sync_W_in,b_rotate_in,b_hard_V_lim_in,b_catch_up_in,direction_in,ts_rollout);

    [State_start,A_max_in,A_min_in] = compensate_global(State_start,A_global,A_max_in,A_min_in);

    P_curr = State_start(:,1);
    V_curr = State_start(:,2);
    A_curr = State_start(:,3);

    for index_waypoint = 1:num_waypoints
        fprintf('Debug: Generating trajectory for waypoint ');printint(index_waypoint);fprintf('!\n');

        % Evolve
        Waypoint_evolved  = evolve_waypoints(Waypoints(:,:,index_waypoint),sum(T_waypoints(:,1:index_waypoint-1),2));

        % Assign Variables
        P_wayp  = Waypoint_evolved(:,1);
        V_wayp  = Waypoint_evolved(:,2);
        A_wayp  = Waypoint_evolved(:,3);
        VP_wayp = Waypoint_evolved(:,4);
        AP_wayp = Waypoint_evolved(:,5);

        V_max = V_max_in(:,index_waypoint);
        V_min = V_min_in(:,index_waypoint);
        A_max = A_max_in(:,index_waypoint);
        A_min = A_min_in(:,index_waypoint);
        J_max = J_max_in(:,index_waypoint);
        J_min = J_min_in(:,index_waypoint);

        b_sync_V     = b_sync_V_in(:,index_waypoint);
        b_sync_A     = b_sync_A_in(:,index_waypoint);
        b_sync_J     = b_sync_J_in(:,index_waypoint);
        b_sync_W     = b_sync_W_in(:,index_waypoint);
        b_rotate     = b_rotate_in(:,index_waypoint);
        b_hard_V_lim = b_hard_V_lim_in(:,index_waypoint);
        b_catch_up   = b_catch_up_in(:,index_waypoint);
        direction    = direction_in(:,index_waypoint);

        % Prediction
        [P_curr_pred,V_curr_pred,A_curr_pred,P_wayp_pred,V_wayp_pred,A_wayp_pred,V_max_pred,V_min_pred,A_max_pred,A_min_pred] = predict_state(P_curr,V_curr,A_curr,P_wayp,V_wayp,A_wayp,VP_wayp,AP_wayp,V_max,V_min,A_max,A_min);

        % Catch up
        T_catch_up = (max(sum(T_waypoints(:,1:index_waypoint-1),2))-sum(T_waypoints(:,1:index_waypoint-1),2)).*b_catch_up;

        % Rotation
        num_rot = num_axes-1;
        theta   = zeros(num_rot,1);
        for index_rot = 1:num_rot
            if (b_rotate(index_rot) == true)
                axis_rot = index_rot + 1;
                theta(index_rot) = atan2((P_wayp_pred(axis_rot) - P_curr_pred(axis_rot)),(P_wayp_pred(1) - P_curr_pred(1)));
                P_curr_pred([1,axis_rot]) = rotate_state(theta(index_rot),P_curr_pred([1,axis_rot]));
                V_curr_pred([1,axis_rot]) = rotate_state(theta(index_rot),V_curr_pred([1,axis_rot]));
                A_curr_pred([1,axis_rot]) = rotate_state(theta(index_rot),A_curr_pred([1,axis_rot]));
                P_wayp_pred([1,axis_rot]) = rotate_state(theta(index_rot),P_wayp_pred([1,axis_rot]));
                V_wayp_pred([1,axis_rot]) = rotate_state(theta(index_rot),V_wayp_pred([1,axis_rot]));
                A_wayp_pred([1,axis_rot]) = rotate_state(theta(index_rot),A_wayp_pred([1,axis_rot]));
            end
        end
        
        % Generate Trajectory
        [t_traj,J_traj,solution_out(:,index_waypoint)] = synchronize_trajectory(P_curr_pred,V_curr_pred,A_curr_pred,P_wayp_pred,V_wayp_pred,A_wayp_pred,V_max_pred,V_min_pred,A_max_pred,A_min_pred,J_max,J_min,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_hard_V_lim,T_catch_up,direction);
        
        % Rotation
        for index_rot = num_rot:-1:1
            if (b_rotate(index_rot) == true)
                axis_rot = index_rot + 1;
                [t_traj{1},J_traj{1},t_traj{axis_rot},J_traj{axis_rot}] = rotate_jerk(theta(index_rot),t_traj{1},J_traj{1},t_traj{axis_rot},J_traj{axis_rot});
            end
        end
        
        for index_axis = 1:num_axes
            t_setp{index_axis,index_waypoint} = t_traj{index_axis};
            J_setp{index_axis,index_waypoint} = J_traj{index_axis};
            T_waypoints(index_axis,index_waypoint) = sum(t_traj{index_axis}(1,:));
            
            %For numerical reasons and computational optimization
            if (VP_wayp(index_axis) == 0 && AP_wayp(index_axis) == 0 && ~isnan(P_wayp(index_axis)) && ~isnan(V_wayp(index_axis)) && ~isnan(A_wayp(index_axis)) && ~solution_out(index_axis,index_waypoint) ~= -1 ) 
                P_curr(index_axis) = Waypoints(index_axis,1,index_waypoint);
                V_curr(index_axis) = Waypoints(index_axis,2,index_waypoint);
                A_curr(index_axis) = Waypoints(index_axis,3,index_waypoint);
            else
                [P_curr(index_axis),V_curr(index_axis),A_curr(index_axis),~] = evaluate_to_time(P_curr(index_axis),V_curr(index_axis),A_curr(index_axis),construct_setp_struct(t_traj{index_axis},J_traj{index_axis}),T_waypoints(index_axis,index_waypoint));
            end
        end
    end
 
    J_setp_struct = construct_setp_struct(t_setp,J_setp);
    [P,V,A,J,t] = rollout_t(State_start(:,1),State_start(:,2),State_start(:,3),J_setp_struct,ts_rollout);
end