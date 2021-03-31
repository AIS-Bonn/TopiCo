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

function check_inputs(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_rotate,b_hard_V_lim,b_catch_up,direction,ts_rollout) %#codegen

    num_axes = size(Waypoints,1);
    num_waypoints = size(Waypoints,3);

     
    validateattributes(State_start,{'double'},{'size',[num_axes,3],'finite','real'},mfilename,'State_start',1);
    validateattributes(Waypoints,{'double'},{'size',[num_axes,5,num_waypoints],'real'},mfilename,'Waypoints',2);
    validateattributes(V_max,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'V_max',3);
    validateattributes(V_min,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'V_min',4);
    validateattributes(A_max,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'A_max',5);
    validateattributes(A_min,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'A_min',6);
    validateattributes(J_max,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'J_max',7);
    validateattributes(J_min,{'double'},{'size',[num_axes,num_waypoints],'nonnan','real'},mfilename,'J_min',8);
    validateattributes(A_global,{'double'},{'size',[num_axes,1],'finite','real'},mfilename,'A_global',9);
    validateattributes(b_sync_V,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_sync_V',10);
    validateattributes(b_sync_A,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_sync_A',11);
    validateattributes(b_sync_J,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_sync_J',12);
    validateattributes(b_sync_W,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_sync_W',13);
    validateattributes(b_rotate,{'logical'},{'size',[num_axes-1,num_waypoints]},mfilename,'b_rotate',14);
    validateattributes(b_hard_V_lim,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_hard_V_lim',15);
    validateattributes(b_catch_up,{'logical'},{'size',[num_axes,num_waypoints]},mfilename,'b_catch_up',16);
    validateattributes(direction,{'int8'},{'size',[num_axes,num_waypoints],'finite',},mfilename,'direction',17);
    validateattributes(ts_rollout,{'double'},{'size',[1,1],'nonnan','positive'},mfilename,'ts_rollout',18);
    

    if (sum(sum(direction == -1 | direction == 0 | direction == 1,1),2)~=num_axes*num_waypoints)
        fprintf('Error: direction is not -1, 0 or +1!\n');
    end

    if (nnz(isinf(Waypoints)) > 0)
        fprintf('Error: Waypoints not allowed to be Inf!\n');
    end
    if (nnz(Waypoints(:,5,:) ~= 0) > 0)
        fprintf('Error: Waypoint acceleration prediction not allowed to be ~= 0!\n');
    end
    if (nnz(isnan(Waypoints(:,4,:))) > 0)
        fprintf('Error: Waypoint velocity prediction not allowed to be NaN!\n');
    end
    if (nnz(isnan(Waypoints(:,5,:))) > 0)
        fprintf('Error: Waypoint acceleration prediction not allowed to be NaN!\n');
    end
    for index_waypoint = 1:num_waypoints
        if (any(sum(isnan(Waypoints(:,1:3,index_waypoint)),2) == 3))
            fprintf('Error: Waypoint needs to be defined by at least one nonNaN in each axis!\n');
        end
    end
    if (nnz(reshape(Waypoints(:,4,:),num_axes,num_waypoints) > V_max) > 0)
        fprintf('Error: Waypoint velocity prediction exceeds V_max!\n');
    end
    
    if (nnz(reshape(Waypoints(:,4,:),num_axes,num_waypoints) < V_min) > 0)
        fprintf('Error: Waypoint velocity prediction exceeds V_min!\n');
    end
    
    if (nnz(reshape(Waypoints(:,5,:),num_axes,num_waypoints) > A_max) > 0)
        fprintf('Error: Waypoint acceleration prediction exceeds A_max!\n');
    end 
     
    if (nnz(reshape(Waypoints(:,5,:),num_axes,num_waypoints) < A_min) > 0)
        fprintf('Error: Waypoint acceleration prediction exceeds A_min!\n');
    end
    

    if (nnz(V_max < 0) > 0)
        fprintf('Error: V_max < 0!\n');
    end
    if (nnz(A_max < 0) > 0)
        fprintf('Error: A_max < 0!\n');
    end
    if (nnz(J_max < 0) > 0)
        fprintf('Error: J_max < 0!\n');
    end
    if (nnz(V_min > 0) > 0)
        fprintf('Error: V_min > 0!\n');
    end
    if (nnz(A_min > 0) > 0)
        fprintf('Error: A_min > 0!\n');
    end
    if (nnz(J_min > 0) > 0)
        fprintf('Error: J_min > 0!\n');
    end

    
    for index_axis = 1:num_axes
        feasibility = check_feasibility(State_start(index_axis,2),State_start(index_axis,3),V_max(index_axis),V_min(index_axis),A_max(index_axis),A_min(index_axis),J_max(index_axis),J_min(index_axis));
        switch feasibility
            case 1
                fprintf('Warning: A_init > A_max!\n');
            case 2
                fprintf('Warning: A_init < A_min!\n');
            case 3
                fprintf('Warning: V_init > V_max!\n');
            case 4
                fprintf('Warning: V_init < V_min!\n');
            case 5
                fprintf('Warning: V_init to close to V_max to prevent future violation of constraints with A_wayp and J_min!\n');
            case 6
                fprintf('Warning: V_init to close to V_min to prevent future violation of constraints with A_wayp and J_max!\n');
            case 7
                fprintf('Warning: V_init to close to V_min and A_init too large (positive) to be reached with J_max!\n');
            case 8
                fprintf('Warning: V_init to close to V_max and A_init too large (negative) to be reached with J_min!\n');
        end
    end
    
    for index_axis = 1:num_axes
        for index_waypoint = 1:num_waypoints
            feasibility = check_feasibility(Waypoints(index_axis,2,index_waypoint)+Waypoints(index_axis,4,index_waypoint),Waypoints(index_axis,3,index_waypoint)+Waypoints(index_axis,5,index_waypoint),V_max(index_axis,index_waypoint),V_min(index_axis,index_waypoint),A_max(index_axis,index_waypoint),A_min(index_axis,index_waypoint),J_max(index_axis,index_waypoint),J_min(index_axis,index_waypoint));
            switch feasibility
                case 1
                    fprintf('Error: Waypoint is infeasible; A_wayp + A_wayp_pred > A_max!\n');
                case 2
                    fprintf('Error: Waypoint is infeasible; A_wayp + A_wayp_pred < A_min!\n');
                case 3
                    fprintf('Error: Waypoint is infeasible; V_wayp + V_wayp_pred > V_max!\n');
                case 4
                    fprintf('Error: Waypoint is infeasible; V_wayp + V_wayp_pred < V_min!\n');
                case 5
                    if (index_waypoint == num_waypoints)
                        fprintf('Warning: V_wayp to close to V_max to prevent future violation of constraints with A_wayp and J_min (when constraints stay the same)!\n');
                    end
                case 6
                    if (index_waypoint == num_waypoints)
                        fprintf('Warning: V_wayp to close to V_min to prevent future violation of constraints with A_wayp and J_max (when constraints stay the same)!\n');
                    end
                case 7
                    fprintf('Error: Waypoint is infeasible; V_wayp to close to V_min and A_wayp too large (positive) to be reached with J_max!\n');
                case 8
                    fprintf('Error: Waypoint is infeasible; V_wayp to close to V_max and A_wayp too large (negative) to be reached with J_min!\n');
            end
        end
    end
    
    for index_axis = 1:num_axes
        for index_waypoint = 2:num_waypoints
            feasibility = check_feasibility(Waypoints(index_axis,2,index_waypoint-1)+Waypoints(index_axis,4,index_waypoint-1),Waypoints(index_axis,3,index_waypoint-1)+Waypoints(index_axis,5,index_waypoint-1),V_max(index_axis,index_waypoint),V_min(index_axis,index_waypoint),A_max(index_axis,index_waypoint),A_min(index_axis,index_waypoint),J_max(index_axis,index_waypoint),J_min(index_axis,index_waypoint));
            switch feasibility
                case 1
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; A_wayp + A_wayp_pred > A_max!\n');
                case 2
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; A_wayp + A_wayp_pred < A_min!\n');
                case 3
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp + V_wayp_pred > V_max!\n');
                case 4
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp + V_wayp_pred < V_min!\n');
                case 5
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp to close to V_max to prevent future violation of constraints with J_min!\n');
                case 6
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp to close to V_min to prevent future violation of constraints with J_max!\n');
                case 7
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp to close to V_min and A_wayp too large (positive) to be reached with J_max!\n');
                case 8
                    fprintf('Warning: Waypoint configuration leads to violation of constraints; V_wayp to close to V_max and A_wayp too large (negative) to be reached with J_min!\n');
            end
        end
    end
    
    if (ts_rollout <= 0)
        fprintf('Error: ts_rollout <= 0!\n');
    end
      
    if (num_axes < 2 && nnz(b_rotate == true) > 0)
        fprintf('Error: num_axes has to be at least 2 when b_rotate == true!\n');
    end


end








