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

function [t] = acdefg_NO_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = J_min.*V_init.*2.0;
l4 = J_min.*V_max.*2.0;
l5 = -J_max;
l7 = sqrt(J_max);
l6 = -l3;
l8 = l7.^3;
l9 = J_min+l5;
l10 = J_min.*l7;
l12 = l2+l4+l6;
l14 = -1.0./(l8-l10);
l15 = l9.*l12;
l16 = -l15;
l17 = sqrt(complex(l16));
l18 = l14.*l17;
t3 = [l18;l17./(l8-l10)];

l2 = A_init.^2;
l3 = A_min.^2;
l5 = A_wayp.^2;
l6 = J_min.^2;
l7 = J_max.^2;
l8 = J_max.^3;
l10 = J_max.^5;
l11 = t3.^2;
l12 = t3.^3;
l4 = l3.^2;
l9 = l7.^2;
l13 = l11.^2;
t4 = ((l4.*l6-l4.*l7+l2.^2.*l7.*3.0-l5.^2.*l6.*3.0+l7.^3.*l13.*3.0+A_min.*l10.*l12.*4.0-J_min.*l10.*l13.*6.0-A_init.^3.*A_min.*l7.*8.0+A_min.*A_wayp.^3.*l6.*8.0+l2.*l3.*l7.*6.0-l3.*l5.*l6.*6.0-l2.*l9.*l11.*6.0-l3.*l9.*l11.*6.0+l6.*l9.*l13.*3.0+V_init.^2.*l6.*l7.*1.2e+1-V_wayp.^2.*l6.*l7.*1.2e+1+J_min.*l2.*l8.*l11.*6.0+J_min.*l3.*l8.*l11.*6.0-V_init.*l6.*l8.*l11.*1.2e+1-A_min.*J_min.*l9.*l12.*1.2e+1-A_min.*P_init.*l6.*l7.*2.4e+1+A_min.*P_wayp.*l6.*l7.*2.4e+1-J_min.*V_init.*l2.*l7.*1.2e+1-J_min.*V_init.*l3.*l7.*1.2e+1+J_min.*V_init.*l9.*l11.*1.2e+1+J_max.*V_wayp.*l3.*l6.*1.2e+1+J_max.*V_wayp.*l5.*l6.*1.2e+1+A_min.*l6.*l8.*l12.*8.0-A_min.*l2.*l8.*t3.*1.2e+1+A_min.*J_min.*V_init.*l8.*t3.*2.4e+1+A_min.*J_min.*l2.*l7.*t3.*1.2e+1-A_min.*V_init.*l6.*l7.*t3.*2.4e+1+A_init.*A_min.*J_min.*V_init.*l7.*2.4e+1-A_min.*A_wayp.*J_max.*V_wayp.*l6.*2.4e+1).*(-1.0./1.2e+1))./(A_min.*J_min.*l2.*l7-A_min.*J_min.*l9.*l11-A_min.*V_init.*l6.*l7.*2.0+A_min.*l6.*l8.*l11);

l2 = A_min.^2;
l3 = t3.^2;
t6 = (J_min.*l2-J_max.*l2+A_init.^2.*J_max-A_wayp.^2.*J_min-J_max.^3.*l3-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_wayp.*2.0+J_min.*J_max.^2.*l3)./(A_min.*J_min.*J_max.*2.0);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

t1 = -(A_init+J_max.*t3)./J_min;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


