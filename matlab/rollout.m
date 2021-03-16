%------------------------------------------------------------------------
% File:       rollout.m
% Version:    2018-06-12 15:24:37
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

function [P,V,A,J] = rollout(P_init,V_init,A_init,J_setp_struct,T,ts) %#codegen

    num_axes = size(J_setp_struct,2);
   
    iterations = round(T/ts);       %induces small rounding error
    
    P.time = 0:ts:ts*iterations;
    V.time = 0:ts:ts*iterations;
    A.time = 0:ts:ts*iterations;
    J.time = 0:ts:ts*iterations;
     
    P.signals.values = zeros(iterations+1,1);
    V.signals.values = zeros(iterations+1,1);
    A.signals.values = zeros(iterations+1,1);
    J.signals.values = zeros(iterations+1,1);
        
    P = repmat(P,1,num_axes);
    V = repmat(V,1,num_axes);
    A = repmat(A,1,num_axes);
    J = repmat(J,1,num_axes);
    
    for index_axis = 1:num_axes
        for index = 0:1:iterations
            t_rollout = index * ts;
            [P(index_axis).signals.values(index+1,1),V(index_axis).signals.values(index+1,1),A(index_axis).signals.values(index+1,1),J(index_axis).signals.values(index+1,1)] = evaluate_to_time(P_init(index_axis),V_init(index_axis),A_init(index_axis),J_setp_struct(index_axis),t_rollout);
        end
        
    end       
        
end