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

function [t] = abcde_NO_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_min.^2;
l5 = J_min.^2;
l6 = J_max.^2;
l4 = l3.^2;
t4 = ((-l4.*l5+l4.*l6-l2.^2.*l6.*3.0+A_init.^3.*A_min.*l6.*8.0+A_min.*A_wayp.^3.*l6.*4.0-l2.*l3.*l6.*6.0-V_init.^2.*l5.*l6.*1.2e+1+V_max.^2.*l5.*l6.*1.2e+1+A_min.*P_init.*l5.*l6.*2.4e+1-A_min.*P_wayp.*l5.*l6.*2.4e+1+J_min.*V_init.*l2.*l6.*1.2e+1+J_min.*V_init.*l3.*l6.*1.2e+1-J_max.*V_max.*l3.*l5.*1.2e+1-A_init.*A_min.*J_min.*V_init.*l6.*2.4e+1+A_min.*A_wayp.*J_min.*V_max.*l6.*2.4e+1).*(-1.0./2.4e+1))./(A_min.*V_max.*l5.*l6);

t5 = A_wayp./J_min;

l2 = A_min.^2;
t2 = (-l2-J_min.*V_init.*2.0+J_min.*V_max.*2.0+A_init.^2+(J_min.*l2)./J_max)./(A_min.*J_min.*2.0);

t3 = -A_min./J_max;

t1 = -(A_init-A_min)./J_min;

t7 = 0.0;

t6 = 0.0;

t = [t1, t2, t3, t4, t5, t6, t7];

end
