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

function [t] = abcef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,~,~,~,A_max,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 18:40:52
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l9 = A_max.^3;
l11 = J_min.^2;
l12 = J_max.^2;
l13 = V_init.^2;
l14 = V_wayp.^2;
l15 = -A_max;
l16 = sqrt(3.0);
l20 = A_min.*J_min.*J_max.*V_wayp.*6.0;
l21 = A_max.*J_min.*J_max.*V_wayp.*6.0;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l18 = A_min+l15;
l19 = J_max.*l6.*3.0;
l22 = J_min.*J_max.*l5;
l23 = A_min.*J_min.*J_max.*l15;
l24 = A_min.*J_max.*l8.*3.0;
l25 = A_max.*J_max.*l5.*6.0;
l32 = A_min.*A_max.*l3.*l11.*8.0;
l33 = l5.*l9.*l12.*3.0;
l34 = l6.*l8.*l12.*3.0;
l35 = A_init.*A_min.*A_max.*J_max.*V_init.*l11.*2.4e+1;
l38 = A_min.*A_max.*P_init.*l11.*l12.*2.4e+1;
l39 = A_min.*A_max.*P_wayp.*l11.*l12.*2.4e+1;
l40 = A_min.*J_max.*V_init.*l2.*l11.*1.2e+1;
l41 = A_min.*J_max.*V_init.*l8.*l11.*1.2e+1;
l42 = A_min.*l2.*l8.*l11.*6.0;
l45 = A_min.*l11.*l12.*l13.*1.2e+1;
l46 = A_max.*l11.*l12.*l14.*1.2e+1;
l26 = A_min.*l10.*l11;
l27 = A_min.*l10.*l12;
l28 = A_max.*l7.*l12;
l29 = A_min.*l4.*l11.*3.0;
l36 = -l34;
l37 = -l35;
l43 = -l39;
l44 = -l42;
l47 = l22+l23;
l48 = -l45;
l30 = -l29;
l31 = -l27;
l49 = 1.0./l47;
l50 = l26+l28+l30+l31+l32+l33+l36+l37+l38+l40+l41+l43+l44+l46+l48;
l51 = l18.*l50;
l52 = -l51;
l53 = sqrt(complex(l52));
l54 = l16.*l53;
t6 = [l49.*(l19-l20+l21+l24-l25-l54).*(-1.0./6.0);l49.*(l19-l20+l21+l24-l25+l54).*(-1.0./6.0)];

l2 = A_max.^2;
t2 = ((J_min.*l2-J_max.*l2-A_init.^2.*J_min+A_min.^2.*J_max+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_wayp.*2.0+A_min.*J_min.*J_max.*t6.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

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

t7 = [0.0;0.0];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

