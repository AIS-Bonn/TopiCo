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

function [t] = abcefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,A_max,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:13:20
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l9 = A_max.^3;
l11 = A_wayp.^2;
l12 = A_wayp.^3;
l14 = J_min.^2;
l15 = J_max.^2;
l16 = V_init.^2;
l17 = V_wayp.^2;
l18 = -A_max;
l19 = 1.0./A_min;
l20 = 1.0./J_min;
l21 = 1.0./J_max;
l22 = sqrt(3.0);
l26 = A_min.*J_min.*J_max.*V_wayp.*6.0;
l27 = A_max.*J_min.*J_max.*V_wayp.*6.0;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l13 = l11.^2;
l23 = A_min+l18;
l24 = J_min.*l6.*3.0;
l25 = J_max.*l6.*3.0;
l28 = A_max.*J_min.*l5.*3.0;
l29 = A_min.*J_max.*l8.*3.0;
l30 = A_max.*J_max.*l5.*6.0;
l31 = A_min.*J_min.*l11.*3.0;
l32 = A_max.*J_min.*l11.*3.0;
l43 = A_min.*A_max.*l3.*l14.*8.0;
l44 = A_min.*A_max.*l12.*l14.*8.0;
l45 = l5.*l9.*l15.*3.0;
l46 = l6.*l8.*l15.*3.0;
l47 = A_init.*A_min.*A_max.*J_max.*V_init.*l14.*2.4e+1;
l48 = A_min.*A_max.*A_wayp.*J_max.*V_wayp.*l14.*2.4e+1;
l52 = A_min.*A_max.*P_init.*l14.*l15.*2.4e+1;
l53 = A_min.*A_max.*P_wayp.*l14.*l15.*2.4e+1;
l54 = A_min.*J_max.*V_init.*l2.*l14.*1.2e+1;
l55 = A_min.*J_max.*V_init.*l8.*l14.*1.2e+1;
l56 = A_max.*J_max.*V_wayp.*l5.*l14.*1.2e+1;
l57 = A_max.*J_max.*V_wayp.*l11.*l14.*1.2e+1;
l58 = A_min.*l2.*l8.*l14.*6.0;
l59 = A_max.*l5.*l11.*l14.*6.0;
l64 = A_min.*l14.*l15.*l16.*1.2e+1;
l65 = A_max.*l14.*l15.*l17.*1.2e+1;
l33 = A_min.*l10.*l14;
l34 = A_max.*l7.*l14;
l35 = A_min.*l10.*l15;
l37 = 1.0./l23;
l38 = A_min.*l4.*l14.*3.0;
l39 = A_max.*l13.*l14.*3.0;
l69 = l23.*(-l33+l34+l35+l38-l39-l43+l44-l45+l46+l47-l48-l52+l53-l54-l55+l56+l57+l58-l59+l64-l65+l7.*l15.*l18);
l70 = sqrt(complex(l69));
l71 = l22.*l70;
t6 = [l19.*l20.*l21.*l37.*(-l24+l25-l26+l27+l28+l29-l30+l31-l32+l71).*(-1.0./6.0);(l19.*l20.*l21.*l37.*(l24-l25+l26-l27-l28-l29+l30-l31+l32+l71))./6.0];

l2 = A_min.^2;
l3 = A_max.^2;
t2 = ((-J_min.*l2+J_min.*l3+J_max.*l2-J_max.*l3-A_init.^2.*J_min+A_wayp.^2.*J_min+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_wayp.*2.0+A_min.*J_min.*J_max.*t6.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = -A_max;
l3 = 1.0./J_min;
l4 = A_min+l2;
l5 = l3.*l4;
t3 = [l5;l5];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


