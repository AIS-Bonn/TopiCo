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

function [t] = acdefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_min.*J_max;
l5 = J_max.*V_init.*2.0;
l6 = J_max.*V_max.*2.0;
l7 = -J_max;
l8 = -l5;
l9 = -l4;
l10 = J_min+l7;
l11 = l3+l9;
l13 = l2+l6+l8;
l12 = 1.0./l11;
l14 = J_min.*l10.*l13;
l15 = sqrt(complex(l14));
l16 = l12.*l15;
t3 = [l16;-l16];

l2 = A_init.^2;
l3 = A_min.^2;
l5 = A_wayp.^2;
l6 = J_min.^2;
l7 = J_min.^3;
l9 = J_max.^2;
l10 = J_min.^5;
l11 = t3.^2;
l12 = t3.^3;
l4 = l3.^2;
l8 = l6.^2;
l13 = l11.^2;
t4 = ((l4.*l6-l4.*l9+l2.^2.*l6.*3.0-l5.^2.*l6.*3.0+l6.^3.*l13.*3.0+A_min.*l10.*l12.*4.0-J_max.*l10.*l13.*6.0-A_init.^3.*A_min.*l6.*8.0+A_min.*A_wayp.^3.*l6.*8.0-l3.*l5.*l6.*6.0-l2.*l8.*l11.*6.0+l8.*l9.*l13.*3.0+V_init.^2.*l6.*l9.*1.2e+1-V_wayp.^2.*l6.*l9.*1.2e+1+J_max.*l2.*l7.*l11.*6.0-J_max.*l3.*l7.*l11.*6.0-V_init.*l7.*l9.*l11.*1.2e+1+l3.*l6.*l9.*l11.*6.0-A_min.*J_max.*l8.*l12.*1.2e+1-A_min.*P_init.*l6.*l9.*2.4e+1+A_min.*P_wayp.*l6.*l9.*2.4e+1+J_min.*J_max.*l2.*l3.*6.0-J_max.*V_init.*l2.*l6.*1.2e+1-J_min.*V_init.*l3.*l9.*1.2e+1+J_max.*V_init.*l8.*l11.*1.2e+1+J_max.*V_wayp.*l3.*l6.*1.2e+1+J_max.*V_wayp.*l5.*l6.*1.2e+1+A_min.*l7.*l9.*l12.*8.0-A_min.*l2.*l7.*t3.*1.2e+1+A_min.*J_max.*V_init.*l7.*t3.*2.4e+1+A_min.*J_max.*l2.*l6.*t3.*1.2e+1-A_min.*V_init.*l6.*l9.*t3.*2.4e+1+A_init.*A_min.*J_max.*V_init.*l6.*2.4e+1-A_min.*A_wayp.*J_max.*V_wayp.*l6.*2.4e+1).*(-1.0./1.2e+1))./(A_min.*l6.*(J_max.*l2-V_init.*l9.*2.0-J_max.*l6.*l11+J_min.*l9.*l11));

l2 = t3.^2;
l3 = -A_wayp;
l4 = A_min+l3;
t6 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0-A_min.*l4.*2.0+J_min.^2.*l2-A_init.^2+l4.^2-J_min.*J_max.*l2+(A_min.^2.*J_max)./J_min).*(-1.0./2.0))./(A_min.*J_max);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

t1 = -(A_init+J_min.*t3)./J_max;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
