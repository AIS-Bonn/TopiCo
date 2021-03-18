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

function [P,V,A] = evaluate(t,J,P_init,V_init,A_init) %#codegen

    num_setpoints = size(t,2);
    
    P = repmat(P_init + t(1) .* V_init + 1/2 .* t(1).^2 .* A_init + 1/6 .* t(1)^3 .* J(:,1),1,num_setpoints);
    V = repmat(V_init + t(1) .* A_init + 1/2 .* t(1).^2 .* J(:,1),1,num_setpoints);
    A = repmat(A_init + t(1) .* J(:,1),1,num_setpoints);

    for index_setpoint = 2:num_setpoints
        t_curr = t(index_setpoint);
        J_curr = J(:,index_setpoint);
        P(:,index_setpoint) = P(:,index_setpoint-1) + t_curr .* V(:,index_setpoint-1) + 1/2 .* t_curr^2 .* A(:,index_setpoint-1) + 1/6 .* t_curr^3 .* J_curr;
        V(:,index_setpoint) = V(:,index_setpoint-1) + t_curr .* A(:,index_setpoint-1) + 1/2 .* t_curr^2 .* J_curr;
        A(:,index_setpoint) = A(:,index_setpoint-1) + t_curr .* J_curr;
    end

end