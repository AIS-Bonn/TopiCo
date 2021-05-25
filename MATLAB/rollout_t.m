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

function [P,V,A,J,t] = rollout_t(P_init,V_init,A_init,J_setp_struct,ts,T) %#codegen
    
    num_axes = size(J_setp_struct,2);

    if (nargin < 6)
        T = 0;
        for index_axis = 1:num_axes
            T = max(T,J_setp_struct(index_axis).time(end));
        end
    end
    
    ts_warning_threshold = 0.001;
    if (ts < ts_warning_threshold)
      fprintf('Warning: Very small ts!\n');
    end
    
    num_iterations = uint32(ceil(T/ts)+1); %Better to make one iteration too much -> Trajectory rollout never ends before reaching final waypoint
    
    if isinf(ts)
        num_iterations = uint32(1);
        ts = 1.0;
    end
    
    num_iterations_warning_threshold = uint32(10000);
    if (num_iterations > num_iterations_warning_threshold)
        fprintf('Warning: Very many rollout iterations (');printint(num_iterations); fprintf(' > ');printint(num_iterations_warning_threshold);fprintf(')!\n');
    
        num_iterations = uint32(double(num_iterations) * double(num_iterations_warning_threshold) / double(num_iterations));
    end
 
    P = zeros(num_axes,num_iterations);
    V = zeros(num_axes,num_iterations);
    A = zeros(num_axes,num_iterations);
    J = zeros(num_axes,num_iterations);
    t = repmat(0.0:ts:ts*double(num_iterations-1),num_axes,1);

    for index_axis = 1:num_axes
        for index_iteration = 1:num_iterations
            [P(index_axis,index_iteration),V(index_axis,index_iteration),A(index_axis,index_iteration),J(index_axis,index_iteration)] = evaluate_to_time(P_init(index_axis),V_init(index_axis),A_init(index_axis),J_setp_struct(index_axis),t(1,index_iteration));
        end
    end
   
end