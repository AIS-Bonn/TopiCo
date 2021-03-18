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

function [feasibility] = check_feasibility(V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min) %#codegen

    V_max      =  1.000000000001 * V_max;  %numerical
    V_min      =  1.000000000001 * V_min;  %numerical
    A_max      =  1.000000000001 * A_max;  %numerical
    A_min      =  1.000000000001 * A_min;  %numerical
    
    num_t = size(V_wayp,2);
    feasibility = uint8(zeros(num_t,1));
    
    V_crit1 = max_V_from_A(A_wayp,V_wayp,-J_min);
    V_crit2 = max_V_from_A(A_wayp,V_wayp,-J_max);
   
    for index = 1:num_t
        if (A_wayp(index) > A_max(index))
            feasibility(index) = 1;
        elseif (A_wayp(index) < A_min(index))
            feasibility(index) = 2;
        elseif (V_wayp(index) > V_max(index))
            feasibility(index) = 3;
        elseif (V_wayp(index) < V_min(index))
            feasibility(index) = 4;
        elseif (V_crit1(index) > V_max(index) && A_wayp(index) >= 0)
            feasibility(index) = 5;
        elseif (V_crit2(index) < V_min(index) && A_wayp(index) < 0)
            feasibility(index) = 6;
        elseif (V_crit2(index) < V_min(index) && A_wayp(index) >= 0)
            feasibility(index) = 7;
        elseif (V_crit1(index) > V_max(index) && A_wayp(index) < 0)
            feasibility(index) = 8;
        end
    end
end