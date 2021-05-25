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

function [t_out,J_out,solution_out] = synchronize_trajectory(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,b_sync_V,b_sync_A,b_sync_J,b_sync_W,b_hard_V_lim,T_catch_up,direction) %#codegen

    num_axes = size(P_init,1);
    
    solution_out = -1 * ones(num_axes,1,'int32');
    
    num_times_red = 4;
    num_times_opt = 7;
    num_times_tim = 11;
    num_times_out = 7;
    num_times     = 11;
    
    
    for index_axis = 1:num_axes %TODO This workaround sometimes prevents numerical issues. Instead, we should find symbolic solutions for the J_max == -J_min and A_max = -A_min condition 
        if (J_max(index_axis) == -J_min(index_axis))
             J_min(index_axis) = -J_max(index_axis)*0.98;
        end
        if (A_max(index_axis) == -A_min(index_axis))
            A_min(index_axis) = A_min(index_axis)*0.98;
        end
    end
     
    t_red = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        t_red{index_axis} = zeros(0,num_times_red );
    end
    J_red = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        J_red{index_axis} = zeros(0,num_times_red );
    end
    t_opt = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        t_opt{index_axis} = zeros(0,num_times_opt);
    end
    J_opt = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        J_opt{index_axis} = zeros(0,num_times_opt);
    end
    t_tim = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        t_tim{index_axis} = zeros(0,num_times_tim);
    end
    J_tim = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        J_tim{index_axis} = zeros(0,num_times_tim);
    end
    t_out = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        t_out{index_axis} = zeros(0,num_times_out);
    end
    J_out = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        J_out{index_axis} = zeros(0,num_times_out);
    end
    t = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        t{index_axis} = zeros(1,num_times);
    end
    J = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        J{index_axis} = zeros(1,num_times);
    end

    solution_opt = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        solution_opt{index_axis} = int32(-1);
    end
    solution_tim = cell(num_axes,1);
    for index_axis = 1:num_axes %coder
        solution_tim{index_axis} = int32(-1);
    end
    
    for index_axis = 1:num_axes
        if (isinf(J_max(index_axis)))
            J_max(index_axis) = 10^6;
        end
        if (isinf(J_min(index_axis)))
            J_min(index_axis) = -10^6;
        end
    end
   
    %% Correct Feasibility
    %parfor index_axis = 1:num_axes
    for index_axis = 1:num_axes
       fprintf('Debug: Ax[');printint(index_axis);fprintf('] Correcting feasibility!\n');
       [t_red{index_axis},J_red{index_axis}] = correct_feasibility(P_init(index_axis),V_init(index_axis),A_init(index_axis),V_max(index_axis),V_min(index_axis),A_max(index_axis),A_min(index_axis),J_max(index_axis),J_min(index_axis),b_hard_V_lim(index_axis));
       [P_init(index_axis),V_init(index_axis),A_init(index_axis)] = evaluate_to_time(P_init(index_axis),V_init(index_axis),A_init(index_axis),construct_setp_struct(t_red{index_axis},J_red{index_axis}));
    end

    %% Find optimal solution
    %parfor index_axis = 1:num_axes
    for index_axis = 1:num_axes
        fprintf('Debug: Ax[');printint(index_axis);fprintf('] Generating optimal solutions!\n');
        [t_opt{index_axis,1},J_opt{index_axis,1},solution_opt{index_axis,1}] = solve_O(P_init(index_axis),V_init(index_axis),A_init(index_axis),P_wayp(index_axis),V_wayp(index_axis),A_wayp(index_axis),V_max(index_axis),V_min(index_axis),A_max(index_axis),A_min(index_axis),J_max(index_axis),J_min(index_axis));
        
        if isempty(solution_opt{index_axis,1}) %So that coder does not crash
            fprintf('Error: This usually can NOT happen. Using idle trajectory to prevent segfault!\n');
            t_opt{index_axis,1} = zeros(1,num_times_opt);
            J_opt{index_axis,1} = zeros(1,num_times_opt);
            solution_opt{index_axis,1} = int32(-1);
        end
    end


    %% Find timed solution
    if (num_axes > 1 && any(b_sync_V == true | b_sync_A == true | b_sync_J == true | b_sync_W == true))
        t_opt_test = t_opt;
        J_opt_test = J_opt;
        solution_opt_test = solution_opt;
        max_iterations = 0;
        for index_axis = 1:num_axes
            max_iterations = max_iterations+size(t_opt_test{index_axis},1);
        end
        fprintf('Debug: Starting synchronization!\n');
        for index_iteration = 1:max_iterations
            T = zeros(num_axes,1);
            for index_axis = 1:num_axes
                T(index_axis) = sum(t_opt_test{index_axis}(1,:),2);
            end
            [~,axis_max] = max(T - T_catch_up);
            t_sync = T(axis_max) + T_catch_up - T_catch_up(axis_max);
            fprintf('Debug: Ax[');printint(index_axis);fprintf('] is the slowest axis!\n');
            for index_axis = 1:num_axes
                if (index_axis ~= axis_max && (b_sync_V(index_axis) == true || b_sync_A(index_axis) == true || b_sync_J(index_axis) == true || b_sync_W(index_axis) == true))
                    if (direction(index_axis) == 0 && (isnan(P_wayp(index_axis)) || isnan(V_wayp(index_axis)) || isnan(A_wayp(index_axis))))
                        fprintf('Debug: Ax[');printint(index_axis);fprintf('] Replacing NaNs with values from optimal solution!\n');
                        [P_wayp(index_axis),V_wayp(index_axis),A_wayp(index_axis),~] = evaluate_to_time(P_init(index_axis),V_init(index_axis),A_init(index_axis),construct_setp_struct(t_opt_test{index_axis}(1,:),J_opt_test{index_axis}(1,:)));
                        fprintf('Debug: Ax[');printint(index_axis);fprintf('] Finding possible multiple solutions for updated state!\n');
                        [t_opt_test{index_axis},J_opt_test{index_axis},solution_opt_test{index_axis}] = solve_O(P_init(index_axis),V_init(index_axis),A_init(index_axis),P_wayp(index_axis),V_wayp(index_axis),A_wayp(index_axis),V_max(index_axis),V_min(index_axis),A_max(index_axis),A_min(index_axis),J_max(index_axis),J_min(index_axis));     
                    end
                    fprintf('Debug: Ax[');printint(index_axis);fprintf('] Searching for timed solutions!\n');
                    [t_tim{index_axis,1},J_tim{index_axis,1},solution_tim{index_axis,1}] = solve_T(P_init(index_axis),V_init(index_axis),A_init(index_axis),P_wayp(index_axis),V_wayp(index_axis),A_wayp(index_axis),V_max(index_axis),V_min(index_axis),A_max(index_axis),A_min(index_axis),J_max(index_axis),J_min(index_axis),t_sync(index_axis),b_sync_V(index_axis),b_sync_A(index_axis),b_sync_J(index_axis),b_sync_W(index_axis),direction(index_axis));
                    if (~isempty(solution_tim{index_axis}))
                        fprintf('Debug: Ax[');printint(index_axis);fprintf('] Found timed solution!\n');
                        t{index_axis,:} = t_tim{index_axis}(1,:);
                        J{index_axis,:} = J_tim{index_axis}(1,:);
                        solution_out(index_axis,1) = solution_tim{index_axis}(1,:);
                    else
                        fprintf('Debug: Ax[');printint(index_axis);fprintf('] Considering slower solution!\n');
                        t_opt_test{index_axis} = circshift(t_opt_test{index_axis},-1);
                        J_opt_test{index_axis} = circshift(J_opt_test{index_axis},-1);
                        solution_opt_test{index_axis} = circshift(solution_opt_test{index_axis},-1);
                        t_opt_test{index_axis}(end,:) = -Inf;
                        J_opt_test{index_axis}(end,:) =  Inf;
                        solution_opt_test{index_axis}(end,:) = -1;
                    break;
                    end
                else
                    t{index_axis,:} = [t_opt_test{index_axis,:}(1,:),zeros(1,4)];
                    J{index_axis,:} = [J_opt_test{index_axis,:}(1,:),zeros(1,4)];
                    solution_out(index_axis,1) = solution_opt_test{index_axis,:}(1,:);
                end
            end
            
            solution_opt_test_all = -1 * ones(100,1,'int32');
            idx = 1;
            for index_axis = 1:num_axes
                size_solution_opt_test = size(solution_opt_test{index_axis},1);
                solution_opt_test_all(idx:idx+size_solution_opt_test-1,1) = solution_opt_test{index_axis};
                idx = idx+size_solution_opt_test;
            end
            solution_opt_test_all = solution_opt_test_all(1:idx-1);
            
            if (all(solution_out(:,1) ~= -1))
                fprintf('Debug: Found timed solutions for all ');printint(num_axes);fprintf(' axes after ');printint(index_iteration);fprintf(' iterations. Exiting synchronization!\n');
                break;
            elseif (nnz(solution_opt_test_all ~= -1) == 0)       
                fprintf('Error: Could not find a timed solution after ');printint(index_iteration);fprintf(' iterations. Exiting synchronization!\n');
                for index_axis = 1:num_axes
                    t{index_axis,:} = [t_opt{index_axis,:}(1,:),zeros(1,4)];
                    J{index_axis,:} = [J_opt{index_axis,:}(1,:),zeros(1,4)];
                    solution_out(index_axis,1) = solution_opt{index_axis,:}(1,:);
                end
                break;
            elseif (index_iteration == max_iterations)
                fprintf('Error: Could not find a timed solution after maximum iterations (');printint(max_iterations);fprintf('). Exiting synchronization!\n');
                for index_axis = 1:num_axes
                    t{index_axis,:} = [t_opt{index_axis,:}(1,:),zeros(1,4)];
                    J{index_axis,:} = [J_opt{index_axis,:}(1,:),zeros(1,4)];
                    solution_out(index_axis,1) = solution_opt{index_axis,:}(1,:);
                end
                break;
            end
        end
    else
        fprintf('Debug: No axis to be synchronized!\n');
        for index_axis = 1:num_axes
            t{index_axis,:} = [t_opt{index_axis,:}(1,:),zeros(1,4)];
            J{index_axis,:} = [J_opt{index_axis,:}(1,:),zeros(1,4)];
            solution_out(index_axis,1) = solution_opt{index_axis,:}(1,:);
        end
    end
    
    for index_axis = 1:num_axes
        t_out{index_axis} = [t_red{index_axis},t{index_axis}(1,:)];
        J_out{index_axis} = [J_red{index_axis},J{index_axis}(1,:)];
    end
end
