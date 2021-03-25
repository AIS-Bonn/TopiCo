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

function [t] = ac_O_P(P_init,V_init,A_init,P_wayp,~,~,V_max,~,~,~,J_max,J_min) %#codegen
% Generated on 17-Sep-2019 12:00:55
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = J_min.^2;
l5 = J_min.^3;
l7 = J_max.^2;
l8 = J_max.^3;
l10 = J_max.^5;
l11 = J_max.*V_init.*2.0;
l12 = J_max.*V_max.*2.0;
l13 = -J_min;
l16 = sqrt(3.0);
l6 = l4.^2;
l9 = l7.^2;
l14 = -l11;
l15 = J_min.*l10;
l17 = J_max+l13;
l18 = P_init.*l10.*6.0;
l19 = P_wayp.*l10.*6.0;
l20 = V_init.*l10.*6.0;
l26 = sqrt(complex(l13));
l33 = l3.*l5.*2.0;
l34 = l3.*l8.*2.0;
l37 = l5.*l8.*3.0;
l38 = A_init.*J_max.*V_init.*l5.*6.0;
l39 = P_init.*l5.*l7.*6.0;
l40 = P_wayp.*l5.*l7.*6.0;
l41 = V_init.*l4.*l8.*6.0;
l42 = V_max.*l5.*l7.*6.0;
l47 = A_init.*J_min.*V_init.*l8.*1.8e+1;
l48 = J_min.*l2.*l8.*6.0;
l49 = J_min.*l3.*l7.*6.0;
l50 = J_max.*l3.*l4.*6.0;
l52 = P_init.*l4.*l8.*1.8e+1;
l53 = P_wayp.*l4.*l8.*1.8e+1;
l55 = V_max.*l4.*l8.*1.2e+1;
l62 = A_init.*V_init.*l4.*l7.*1.8e+1;
l63 = l2.*l4.*l7.*3.0;
l21 = -l19;
l22 = -l20;
l23 = A_init.*V_init.*l9.*6.0;
l24 = J_min.*V_max.*l9.*6.0;
l25 = l6.*l7;
l27 = l26.^3;
l28 = l26.^5;
l30 = J_min.*P_init.*l9.*1.8e+1;
l31 = J_min.*P_wayp.*l9.*1.8e+1;
l32 = J_min.*V_init.*l9.*1.2e+1;
l35 = l2.*l9.*3.0;
l36 = l4.*l9.*3.0;
l44 = -l33;
l51 = -l39;
l54 = -l41;
l56 = sqrt(complex(l17));
l58 = -l48;
l59 = -l49;
l60 = -l53;
l61 = -l55;
l64 = -l62;
l65 = l2+l12+l14;
l29 = -l23;
l43 = -l30;
l45 = -l25;
l46 = -l36;
l57 = l56.^3;
l66 = sqrt(complex(l65));
l85 = l22+l24+l32+l35+l42+l54+l58+l61+l63;
l67 = l66.^3;
l68 = l15+l37+l45+l46;
l73 = J_max.*V_init.*l28.*l56.*l66.*6.0;
l74 = l2.*l28.*l56.*l66.*3.0;
l75 = l9.*l26.*l56.*l66.*3.0;
l76 = l7.*l28.*l56.*l66.*3.0;
l77 = l8.*l27.*l56.*l66.*6.0;
l79 = J_max.*l2.*l27.*l56.*l66.*6.0;
l80 = V_init.*l8.*l26.*l56.*l66.*6.0;
l82 = V_init.*l7.*l27.*l56.*l66.*1.2e+1;
l83 = l2.*l7.*l26.*l56.*l66.*3.0;
l69 = 1.0./l68;
l72 = l27.*l57.*l67;
l78 = -l74;
l81 = -l79;
l84 = -l83;
l87 = l75+l76+l77;
l70 = l69.^2;
l71 = l69.^3;
l86 = (l69.*l85)./3.0;
l88 = l87.^2;
l89 = l87.^3;
l90 = (l69.*l87)./3.0;
l100 = l18+l21+l29+l31+l34+l38+l40+l43+l44+l47+l50+l51+l52+l59+l60+l64+l72+l73+l78+l80+l81+l82+l84;
l91 = -l90;
l92 = (l70.*l88)./9.0;
l93 = (l71.*l89)./2.7e+1;
l95 = (l70.*l85.*l87)./6.0;
l101 = (l69.*l100)./2.0;
l94 = -l93;
l96 = -l95;
l97 = l86+l92;
l102 = -l101;
l103 = l93+l95+l101;
l98 = l97.^3;
l104 = l103.^2;
l99 = -l98;
l105 = l99+l104;
l106 = sqrt(complex(l105));
l107 = l94+l96+l102+l106;
l108 = l107.^(1.0./3.0);
l109 = 1.0./l108;
l110 = l108./2.0;
l111 = -l110;
l112 = l97.*l109;
l113 = -l112;
l114 = l112./2.0;
l115 = -l114;
l116 = l108+l113;
l117 = l16.*l116.*5.0e-1i;
t3 = [l91+l108+l112;l91+l111+l115-l117;l91+l111+l115+l117];

l2 = A_init.^2;
l3 = J_max.^2;
l4 = A_init.*J_min;
l5 = A_init.*J_max;
l6 = J_min.*J_max;
l7 = J_max.*V_init.*2.0;
l8 = J_max.*V_max.*2.0;
l9 = -J_max;
l10 = -l7;
l11 = -l5;
l12 = -l6;
l13 = J_min+l9;
l14 = l3+l12;
l16 = l2+l8+l10;
l15 = 1.0./l14;
l17 = J_min.*l13.*l16;
l18 = sqrt(complex(l17));
l19 = l4+l11+l18;
l20 = l15.*l19;
t1 = [l20;l20;l20];

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
