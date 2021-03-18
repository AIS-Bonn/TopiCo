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

function [P,V,A,J] = evaluate_to_time(P_init,V_init,A_init,J_setp_struct,T_in) %#codegen

    num_axes = size(J_setp_struct,2);
    
    if (nargin < 5)
        T_in = zeros(num_axes,1);
        for index_axis = 1:num_axes
            T_in(index_axis) = J_setp_struct(index_axis).time(end);
        end
    end
    
    if (num_axes > 1 && size(T_in,1) == 1)
        T = repmat(T_in,num_axes,1);
    else
        T = T_in;
    end
    
    P = zeros(num_axes,1);
    V = zeros(num_axes,1);
    A = zeros(num_axes,1);
    J = zeros(num_axes,1);
    
    [t_temp,J_temp] = destruct_setp_struct(J_setp_struct);
    [t_remaining,J_remaining] = cut_to_time(t_temp,J_temp,T);

    for index_axis = 1:num_axes

        [P_eval,V_eval,A_eval] = evaluate(t_remaining{index_axis},J_remaining{index_axis},P_init(index_axis),V_init(index_axis),A_init(index_axis));

        P(index_axis,1) = P_eval(end);
        V(index_axis,1) = V_eval(end);
        A(index_axis,1) = A_eval(end);
        J(index_axis,1) = J_remaining{index_axis}(end);

    end

end
