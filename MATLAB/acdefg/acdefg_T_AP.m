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

function [t] = acdefg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 12:12:49
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l6 = A_wayp.^2;
l7 = A_wayp.^3;
l8 = J_min.^2;
l10 = J_max.^2;
l11 = J_max.*V_init.*2.0;
l12 = J_max.*V_max.*2.0;
l13 = 1.0./A_min;
l14 = -J_min;
l16 = 1.0./J_max;
l18 = sqrt(3.0);
l9 = l8.^2;
l15 = 1.0./l8;
l17 = -l11;
l19 = J_max+l14;
l20 = A_min.*A_wayp.*l8.*6.0;
l21 = J_min.*J_max.*l4.*3.0;
l22 = l14.^(7.0./2.0);
l23 = l4.*l8.*6.0;
l24 = l14.^(9.0./2.0);
l25 = l14.^(1.1e+1./2.0);
l26 = l19.^(3.0./2.0);
l28 = l2+l12+l17;
l27 = 1.0./l26;
l29 = sqrt(complex(l28));
l31 = l5.*l9.*l26.*4.0;
l32 = l7.*l9.*l26.*4.0;
l33 = l3.*l9.*l26.*8.0;
l35 = A_init.*J_max.*V_init.*l9.*l26.*2.4e+1;
l36 = A_init.*J_max.*V_max.*l9.*l26.*2.4e+1;
l37 = l5.*l8.*l10.*l26;
l39 = A_min.*l6.*l9.*l26.*1.2e+1;
l40 = A_wayp.*l4.*l9.*l26.*1.2e+1;
l41 = P_init.*l9.*l10.*l26.*2.4e+1;
l42 = P_wayp.*l9.*l10.*l26.*2.4e+1;
l43 = T.*V_max.*l9.*l10.*l26.*2.4e+1;
l30 = l29.^3;
l34 = -l31;
l38 = -l35;
l44 = -l39;
l45 = -l42;
l48 = J_max.*V_init.*l25.*l29.*2.4e+1;
l49 = J_max.*V_max.*l25.*l29.*2.4e+1;
l52 = l2.*l25.*l29.*1.2e+1;
l54 = J_max.*l2.*l24.*l29.*1.2e+1;
l55 = V_init.*l10.*l24.*l29.*2.4e+1;
l56 = V_max.*l10.*l24.*l29.*2.4e+1;
l46 = l25.*l30.*4.0;
l47 = l10.*l22.*l30.*4.0;
l51 = -l49;
l53 = -l52;
l57 = -l54;
l58 = -l56;
l50 = -l47;
l59 = l32+l33+l34+l36+l37+l38+l40+l41+l43+l44+l45+l46+l48+l50+l51+l53+l55+l57+l58;
l60 = A_min.*l27.*l59;
l61 = -l60;
l62 = sqrt(complex(l61));
l63 = l18.*l62;
t6 = [l13.*l15.*l16.*(l20+l21-l23+l63).*(-1.0./6.0);l13.*l15.*l16.*(l20+l21-l23-l63).*(-1.0./6.0)];

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
t3 = [l16;l16];

l2 = A_init.^2;
l3 = A_min.^2;
l4 = A_min.^3;
l5 = J_min.^2;
l6 = J_min.^3;
l8 = J_max.^2;
l9 = t3.^2;
l10 = t3.^3;
l7 = l5.^2;
t4 = (l4.*l5.*2.0+l4.*l8+A_init.^3.*l5.*2.0+A_wayp.^3.*l5-J_min.^5.*l10-J_min.*J_max.*l4.*3.0+A_min.*l2.*l5.*3.0-A_min.*l7.*l9.*3.0-A_wayp.*l2.*l5.*3.0-A_wayp.*l3.*l5.*3.0+A_wayp.*l7.*l9.*3.0+J_max.*l7.*l10.*3.0+P_init.*l5.*l8.*6.0-P_wayp.*l5.*l8.*6.0-l6.*l8.*l10.*2.0+l2.*l6.*t3.*3.0-J_max.*l2.*l5.*t3.*3.0-J_max.*l2.*l5.*t6.*3.0-J_max.*l3.*l5.*t6.*6.0+J_min.*l3.*l8.*t6.*3.0+J_max.*l7.*l9.*t6.*3.0+V_init.*l5.*l8.*t3.*6.0+V_init.*l5.*l8.*t6.*6.0-l6.*l8.*l9.*t6.*3.0+A_min.*l5.*l8.*t6.^2.*3.0-A_min.*J_min.*J_max.*l2.*3.0+A_wayp.*J_min.*J_max.*l3.*3.0-A_init.*J_max.*V_init.*l5.*6.0-A_min.*J_max.*V_init.*l5.*6.0+A_min.*J_min.*V_init.*l8.*6.0+A_wayp.*J_max.*V_init.*l5.*6.0+A_min.*J_max.*l6.*l9.*6.0-A_wayp.*J_max.*l6.*l9.*3.0-J_max.*V_init.*l6.*t3.*6.0-A_min.*l5.*l8.*l9.*3.0+A_min.*A_wayp.*J_max.*l5.*t6.*6.0)./(J_max.*l2.*l5.*3.0-J_max.*l7.*l9.*3.0-V_init.*l5.*l8.*6.0+l6.*l8.*l9.*3.0);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

t1 = -(A_init+J_min.*t3)./J_max;

t5 = -(A_init-A_min+J_min.*t3+J_max.*t1)./J_min;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
