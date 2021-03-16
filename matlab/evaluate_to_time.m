%------------------------------------------------------------------------
% File:       evaluate_to_time.m
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

function [P,V,A,J] = evaluate_to_time(P_init,V_init,A_init,J_setp_struct,T_in) %#codegen

    num_axes = size(J_setp_struct,2);
    
    if (num_axes > 1 && size(T_in,1) == 1)
        T = repmat(T_in,num_axes,1);
    else
        T = T_in;
    end
    
    P = zeros(num_axes,1);
    V = zeros(num_axes,1);
    A = zeros(num_axes,1);
    J = zeros(num_axes,1);

    for index_axis = 1:num_axes

        if (T(index_axis) > J_setp_struct(index_axis).time(end))
            J_setp_struct(index_axis).time = vertcat(J_setp_struct(index_axis).time,T(index_axis));
            J_setp_struct(index_axis).signals.values = horzcat(J_setp_struct(index_axis).signals.values,0);
        end

        index_setp_max = find(J_setp_struct(index_axis).time<=T(index_axis),1,'last');
        
        t_remaining = cat(2,diff(J_setp_struct(index_axis).time(1:index_setp_max(1,1),1))',T(index_axis) - J_setp_struct(index_axis).time(index_setp_max(1,1),1));
        J_remaining = J_setp_struct(index_axis).signals.values(1,1:index_setp_max(1,1));

        [P_eval,V_eval,A_eval] = evaluate(t_remaining,J_remaining,P_init(index_axis),V_init(index_axis),A_init(index_axis));

        P(index_axis,1) = P_eval(end);
        V(index_axis,1) = V_eval(end);
        A(index_axis,1) = A_eval(end);
        J(index_axis,1) = J_remaining(end);

    end
    
end
