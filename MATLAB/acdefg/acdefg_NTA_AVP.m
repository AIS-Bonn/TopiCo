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

function [t] = acdefg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 17-Sep-2019 10:06:26
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_wayp.^2;
l5 = A_wayp.^3;
l6 = J_min.^2;
l7 = J_min.^3;
l9 = J_max.^2;
l10 = J_max.^3;
l12 = J_min.*J_max;
l13 = J_min.*V_init.*2.0;
l14 = J_min.*V_max.*2.0;
l15 = J_max.*V_max.*2.0;
l16 = J_max.*V_wayp.*2.0;
l17 = -J_min;
l18 = -J_max;
l21 = sqrt(3.0);
l22 = sqrt(6.0);
l23 = J_max.^(3.0./2.0);
l24 = J_max.^(5.0./2.0);
l25 = J_max.^(7.0./2.0);
l8 = l6.^2;
l11 = l6.^3;
l19 = -l13;
l20 = -l16;
l26 = -l12;
l28 = J_min+l18;
l29 = J_max+l17;
l31 = l5.*l7.*4.0;
l32 = l5.*l6.*6.0;
l33 = l7.*l9.*3.0;
l35 = A_wayp.*l2.*l12.*6.0;
l36 = V_init.*l6.*l9.*6.0;
l37 = V_wayp.*l6.*l9.*6.0;
l40 = A_init.*J_min.*V_init.*l9.*1.2e+1;
l41 = A_wayp.*J_max.*V_init.*l6.*1.2e+1;
l42 = A_wayp.*J_max.*V_wayp.*l6.*1.2e+1;
l43 = J_min.*l2.*l9.*3.0;
l44 = J_max.*l4.*l6.*3.0;
l46 = P_init.*l7.*l9.*2.4e+1;
l47 = P_wayp.*l6.*l9.*1.2e+1;
l51 = J_min.*l3.*l9.*8.0;
l52 = A_init.*V_max.*l6.*l9.*2.4e+1;
l54 = T.*V_max.*l7.*l9.*2.4e+1;
l27 = l11.*8.0;
l30 = J_max.*l8.*3.0;
l34 = l8.*l9.*4.0;
l38 = 1.0./l28;
l39 = -l35;
l45 = l6+l26;
l48 = -l37;
l49 = -l42;
l50 = -l43;
l55 = 1.0./sqrt(complex(l29));
l57 = l2+l14+l19;
l58 = l4+l15+l20;
l53 = l45.^2;
l56 = l55.^3;
l59 = l8.*l45.*1.2e+1;
l60 = l58.^2;
l63 = l30+l33;
l66 = sqrt(complex(l57));
l69 = l8.*l58.*1.2e+1;
l72 = l6.*l45.*l58.*6.0;
l73 = J_min.*l10.*l38.*l57.*3.0;
l75 = A_wayp.*J_min.*l9.*l38.*l57.*6.0;
l76 = A_wayp.*J_max.*l6.*l38.*l57.*6.0;
l78 = l6.*l9.*l38.*l57.*3.0;
l61 = l60.^2;
l62 = l60.^3;
l64 = -l59;
l65 = l6.*l53.*3.0;
l67 = l66.^3;
l68 = J_min.*l63.*2.0;
l71 = -l69;
l74 = -l73;
l77 = -l75;
l80 = J_min.*V_init.*l24.*l55.*l66.*1.2e+1;
l82 = J_min.*l2.*l24.*l55.*l66.*1.2e+1;
l83 = V_max.*l6.*l24.*l55.*l66.*2.4e+1;
l70 = -l68;
l79 = l25.*l56.*l67.*2.0;
l81 = l7.*l23.*l56.*l67.*4.0;
l93 = l36+l44+l48+l50+l74+l78;
l84 = l27+l34+l64+l65+l70;
l94 = J_min.*l93.*2.0;
l99 = l32+l39+l40+l41+l47+l49+l76+l77+l79+l80;
l85 = 1.0./l84;
l95 = l71+l72+l94;
l100 = J_min.*l99.*2.0;
l86 = l85.^2;
l87 = l85.^3;
l89 = l85.^5;
l90 = l6.*l60.*l85.*3.6e+1;
l96 = l95.^2;
l97 = l95.^3;
l101 = -l100;
l88 = l86.^2;
l91 = l11.*l62.*l87.*6.912e+3;
l98 = l96.^2;
l102 = l86.*l96;
l103 = l87.*l97.*2.0;
l105 = (l87.*l97)./2.7e+1;
l107 = l6.*l60.*l86.*l95.*4.0;
l108 = l6.*l60.*l86.*l95.*2.16e+2;
l112 = l31+l46+l51+l52+l54+l81+l82+l83+l101;
l92 = -l91;
l104 = -l103;
l106 = -l105;
l109 = l6.*l60.*l89.*l98.*4.8e+1;
l111 = l8.*l61.*l88.*l96.*1.152e+3;
l113 = l112.^2;
l110 = -l109;
l114 = l113.^2;
l115 = l86.*l113.*2.7e+1;
l117 = (l86.*l113)./2.0;
l118 = l89.*l97.*l113.*4.0;
l120 = l6.*l60.*l88.*l95.*l113.*4.32e+2;
l116 = l88.*l114.*2.7e+1;
l119 = -l118;
l121 = l92+l110+l111+l116+l119+l120;
l122 = sqrt(complex(l121));
l123 = l21.*l122.*3.0;
l124 = (l21.*l122)./1.8e+1;
l125 = l104+l108+l115+l123;
l127 = l106+l107+l117+l124;
l126 = sqrt(complex(l125));
l128 = l127.^(1.0./3.0);
l130 = 1.0./l127.^(1.0./6.0);
l129 = l128.^2;
l132 = l85.*l95.*l128.*6.0;
l133 = l22.*l85.*l112.*l126.*3.0;
l131 = l129.*9.0;
l134 = -l133;
l135 = l90+l102+l131+l132;
l136 = sqrt(complex(l135));
l137 = 1.0./l135.^(1.0./4.0);
l139 = l6.*l60.*l85.*l136.*-3.6e+1;
l140 = l102.*l136;
l143 = l129.*l136.*-9.0;
l144 = (l130.*l136)./6.0;
l146 = l85.*l95.*l128.*l136.*1.2e+1;
l141 = -l140;
l145 = -l144;
l147 = l133+l139+l141+l143+l146;
l148 = l134+l139+l141+l143+l146;
l149 = sqrt(complex(l147));
l150 = sqrt(complex(l148));
l151 = (l130.*l137.*l149)./6.0;
l152 = (l130.*l137.*l150)./6.0;
t5 = [l145-l151;l145+l151;l144-l152;l144+l152];

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
l15 = l9.*l12;
l16 = -l15;
l17 = sqrt(complex(l16));
l19 = l17./(l8-l10);
t3 = [l19;l19;l19;l19];

l2 = t3.^2;
l3 = t5.^2;
t6 = (1.0./J_min.^2.*(-J_max.*(J_min.*V_wayp.*2.0+A_init.^2+J_min.*J_max.*l2)+A_wayp.^2.*J_min-J_min.^3.*l3+J_max.^3.*l2+J_min.*J_max.*V_init.*2.0+J_min.^2.*J_max.*l3).*(-1.0./2.0))./(J_max.*t5);

l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_min.^3;
l6 = J_max.^2;
l7 = J_max.^3;
l9 = t3.^2;
l10 = t3.^3;
l11 = t5.^2;
l12 = t5.^3;
l5 = l3.^2;
l8 = l6.^2;
t4 = (A_init.^3.*l6.*2.0+A_wayp.^3.*l3+J_min.^5.*l12.*2.0-J_max.^5.*l10-A_wayp.*l5.*l11.*3.0+J_min.*l8.*l10.*3.0-J_max.*l5.*l12.*3.0+P_init.*l3.*l6.*6.0-P_wayp.*l3.*l6.*6.0-l3.*l7.*l10.*2.0+l4.*l6.*l12+l2.*l7.*t3.*3.0-J_min.*l2.*l6.*t3.*3.0+J_max.*l2.*l3.*t5.*3.0-J_min.*l2.*l6.*t5.*3.0-J_min.*l2.*l6.*t6.*3.0+J_min.*l8.*l9.*t5.*3.0+J_min.*l8.*l9.*t6.*3.0-J_max.*l5.*l11.*t6.*6.0+V_init.*l3.*l6.*t3.*6.0+V_init.*l3.*l6.*t5.*6.0+V_init.*l3.*l6.*t6.*6.0-l3.*l7.*l9.*t5.*6.0+l4.*l6.*l9.*t5.*3.0-l3.*l7.*l9.*t6.*3.0+l4.*l6.*l11.*t6.*3.0+l4.*l6.*t5.*t6.^2.*3.0-A_wayp.*J_min.*J_max.*l2.*3.0-A_init.*J_min.*V_init.*l6.*6.0+A_wayp.*J_max.*V_init.*l3.*6.0+A_wayp.*J_min.*l7.*l9.*3.0+A_wayp.*J_max.*l4.*l11.*3.0-J_min.*V_init.*l7.*t3.*6.0-J_max.*V_init.*l4.*t5.*6.0-A_wayp.*l3.*l6.*l9.*3.0+A_wayp.*J_max.*l4.*t5.*t6.*6.0)./(l6.*(J_min.*l2-V_init.*l3.*2.0+J_max.*l3.*l9-J_min.*l6.*l9).*3.0);

t7 = (A_wayp-J_min.*t5)./J_max;

t1 = -(A_init+J_max.*t3)./J_min;

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


