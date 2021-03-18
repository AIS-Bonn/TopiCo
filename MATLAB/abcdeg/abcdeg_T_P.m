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

function [t] = abcdeg_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,V_min,A_max,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 14:20:07
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l6 = A_max.^3;
l8 = J_min.^2;
l9 = J_min.^3;
l11 = J_max.^2;
l12 = J_min.^5;
l13 = J_max.^3;
l15 = J_max.^5;
l17 = V_init.^2;
l18 = V_max.^2;
l19 = -J_min;
l20 = -J_max;
l21 = 1.0./J_min;
l22 = 1.0./J_max;
l23 = -V_min;
l24 = -V_max;
l25 = 1.0./V_max;
l26 = A_init.*A_max.*J_min.*2.0;
l27 = J_min.*J_max.*V_init.*2.0;
l29 = sqrt(2.0);
l30 = sqrt(3.0);
l31 = J_max.^(3.0./2.0);
l32 = J_max.^(5.0./2.0);
l33 = J_max.^(7.0./2.0);
l38 = A_max.*J_min.*J_max.*T.*2.0;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l14 = l11.^2;
l16 = l11.^3;
l28 = J_max.*l5;
l34 = J_min+l20;
l35 = J_max+l19;
l36 = V_min+l24;
l37 = V_max+l23;
l39 = l31.^5;
l42 = sqrt(complex(l19));
l54 = A_max.*l12.*l13.*4.0;
l55 = A_init.*A_max.*J_max.*V_init.*l12.*2.4e+1;
l60 = A_max.*l3.*l12.*8.0;
l65 = A_max.*l9.*l15.*1.2e+1;
l69 = A_max.*P_init.*l8.*l15.*2.4e+1;
l70 = A_max.*P_init.*l11.*l12.*2.4e+1;
l71 = A_max.*P_wayp.*l8.*l15.*2.4e+1;
l72 = A_max.*P_wayp.*l11.*l12.*2.4e+1;
l73 = J_max.*V_init.*l2.*l12.*1.2e+1;
l74 = J_max.*V_max.*l2.*l12.*1.2e+1;
l76 = J_min.*V_max.*l5.*l15.*1.2e+1;
l78 = A_max.*V_min.*l8.*l15.*2.4e+1;
l80 = A_max.*V_max.*l8.*l15.*2.4e+1;
l84 = l2.*l5.*l12.*6.0;
l105 = l8.*l15.*l17.*1.2e+1;
l106 = l11.*l12.*l17.*1.2e+1;
l109 = l8.*l15.*l18.*1.2e+1;
l110 = l11.*l12.*l18.*1.2e+1;
l121 = A_init.*A_max.*V_init.*l9.*l13.*7.2e+1;
l123 = A_max.*l3.*l8.*l13.*8.0;
l125 = A_max.*l3.*l9.*l11.*2.4e+1;
l127 = V_init.*l2.*l9.*l13.*3.6e+1;
l131 = V_max.*l2.*l9.*l13.*3.6e+1;
l133 = V_init.*l5.*l9.*l13.*3.6e+1;
l145 = l2.*l5.*l8.*l13.*6.0;
l146 = l2.*l5.*l9.*l11.*1.8e+1;
l40 = l7.*l12;
l41 = l7.*l15;
l43 = l42.^3;
l44 = l42.^5;
l45 = l4.*l12.*3.0;
l46 = l34.^3;
l47 = l34.^5;
l51 = J_min.*l7.*l14.*3.0;
l52 = J_max.*l7.*l10.*3.0;
l53 = A_max.*l8.*l16.*4.0;
l56 = l35.^(3.0./2.0);
l57 = l35.^(5.0./2.0);
l58 = sqrt(complex(l37));
l61 = J_max.*l4.*l10.*9.0;
l64 = -l54;
l66 = A_max.*l10.*l14.*1.2e+1;
l67 = -l55;
l68 = A_max.*J_max.*l3.*l10.*2.4e+1;
l75 = V_init.*l12.*l28.*1.2e+1;
l77 = V_max.*l12.*l28.*1.2e+1;
l79 = A_max.*V_min.*l10.*l13.*2.4e+1;
l81 = A_max.*V_max.*l10.*l13.*2.4e+1;
l82 = A_max.*V_min.*l9.*l14.*4.8e+1;
l83 = A_max.*V_max.*l9.*l14.*4.8e+1;
l85 = l4.*l8.*l13.*3.0;
l86 = l7.*l8.*l13.*2.0;
l87 = l7.*l9.*l11.*2.0;
l88 = 1.0./sqrt(complex(l35));
l90 = -l65;
l92 = -l69;
l93 = A_max.*P_init.*l9.*l14.*7.2e+1;
l94 = A_max.*P_init.*l10.*l13.*7.2e+1;
l95 = -l72;
l96 = A_max.*P_wayp.*l9.*l14.*7.2e+1;
l97 = A_max.*P_wayp.*l10.*l13.*7.2e+1;
l98 = -l74;
l100 = -l80;
l103 = -l84;
l104 = l4.*l9.*l11.*9.0;
l107 = l9.*l14.*l17.*3.6e+1;
l108 = l10.*l13.*l17.*3.6e+1;
l111 = l9.*l14.*l18.*3.6e+1;
l112 = l10.*l13.*l18.*3.6e+1;
l113 = A_init.*A_max.*V_init.*l8.*l14.*2.4e+1;
l117 = -l106;
l119 = -l110;
l122 = A_init.*A_max.*V_init.*l10.*l11.*7.2e+1;
l124 = l2.*l10.*l28.*1.8e+1;
l126 = V_init.*l2.*l8.*l14.*1.2e+1;
l128 = V_init.*l2.*l10.*l11.*3.6e+1;
l129 = V_max.*l2.*l8.*l14.*1.2e+1;
l130 = V_init.*l5.*l8.*l14.*1.2e+1;
l132 = V_max.*l2.*l10.*l11.*3.6e+1;
l134 = V_init.*l5.*l10.*l11.*3.6e+1;
l135 = V_max.*l5.*l8.*l14.*2.4e+1;
l136 = V_max.*l5.*l10.*l11.*2.4e+1;
l137 = -l121;
l138 = -l123;
l142 = -l131;
l147 = -l146;
l48 = l43.^5;
l49 = -l45;
l50 = 1.0./l46;
l59 = l58.^3;
l62 = -l51;
l63 = -l52;
l89 = l56.^5;
l91 = -l68;
l99 = -l77;
l101 = -l81;
l102 = -l82;
l114 = -l94;
l115 = -l96;
l116 = -l104;
l118 = -l107;
l120 = -l111;
l139 = -l126;
l140 = -l128;
l141 = -l130;
l143 = -l134;
l144 = -l135;
l148 = l53+l64+l66+l90;
l149 = A_max.*l29.*l31.*l42.*l58.*l88.*2.0;
l151 = A_max.*V_max.*l29.*l32.*l43.*l57.*l58.*2.4e+1;
l150 = A_max.*l29.*l33.*l43.*l56.*l59.*8.0;
l152 = -l151;
l153 = 1.0./l148;
l157 = l78+l79+l83+l100+l101+l102;
l154 = l153.^2;
l155 = l153.^3;
l158 = A_max.*l29.*l32.*l44.*l57.*l58.*l153.*4.0;
l160 = (l153.*l157)./3.0;
l165 = l40+l41+l49+l60+l61+l62+l63+l67+l70+l71+l73+l75+l76+l85+l86+l87+l91+l92+l93+l95+l97+l98+l99+l103+l105+l108+l109+l112+l113+l114+l115+l116+l117+l118+l119+l120+l122+l124+l125+l127+l129+l132+l133+l136+l137+l138+l139+l140+l141+l142+l143+l144+l145+l147+l150+l152;
l156 = l5.*l12.*l15.*l36.*l47.*l154.*3.2e+1;
l159 = l6.*l29.*l39.*l48.*l59.*l89.*l155.*1.28e+2;
l161 = A_max.*l29.*l32.*l44.*l57.*l58.*l154.*l157.*2.0;
l166 = (l21.*l22.*l25.*l50.*l165)./1.2e+1;
l162 = -l161;
l163 = l156+l160;
l167 = l26+l27+l28+l38+l149+l166;
l164 = l163.^3;
l168 = J_min.*J_max.*V_max.*l46.*l153.*l167.*6.0;
l169 = l159+l162+l168;
l170 = l169.^2;
l171 = l164+l170;
l172 = sqrt(complex(l171));
l173 = l169+l172;
l174 = l173.^(1.0./3.0);
l175 = 1.0./l174;
l176 = l174./2.0;
l177 = -l176;
l178 = l163.*l175;
l179 = l178./2.0;
l180 = l174+l178;
l181 = l30.*l180.*5.0e-1i;
t7 = [l158+l174-l178;l158+l177+l179-l181;l158+l177+l179+l181];

l2 = A_init.^2;
l3 = A_max.^2;
l4 = A_max.^3;
l6 = J_min.^2;
l7 = J_max.^2;
l8 = J_max.^3;
l9 = -A_max;
l10 = -J_min;
l11 = -J_max;
l12 = -V_min;
l13 = J_min.*J_max.*V_init.*2.0;
l14 = J_min.*J_max.*V_max.*2.0;
l18 = sqrt(2.0);
l19 = J_max.^(3.0./2.0);
l20 = J_max.^(5.0./2.0);
l5 = l3.^2;
l15 = J_min.*l2;
l16 = J_min.*l3;
l17 = J_max.*l3;
l21 = A_init+l9;
l22 = -l13;
l23 = J_min+l11;
l24 = J_max+l10;
l25 = V_max+l12;
l26 = l3.*l10;
l27 = sqrt(complex(l10));
l28 = l27.^3;
l29 = l23.^3;
l30 = l24.^(5.0./2.0);
l31 = sqrt(complex(l25));
l32 = l14+l15+l17+l22+l26;
t4 = -(l29.*l32.^2.*3.0+l5.*l7.*l29.*8.0+l15.*l17.*l29.*1.2e+1-l15.*l29.*l32.*6.0+l16.*l29.*l32.*6.0-l17.*l29.*l32.*1.2e+1-V_init.*l7.*l16.*l29.*2.4e+1+A_init.^3.*A_max.*l6.*l29.*1.2e+1-l2.*l3.*l6.*l29.*2.4e+1-A_max.*l6.*l21.^3.*l29.*4.0-J_min.*J_max.*l5.*l29.*1.2e+1+A_init.*l4.*l6.*l29.*1.2e+1+J_min.*J_max.*V_init.*l29.*l32.*1.2e+1+A_max.*P_init.*l6.*l7.*l29.*2.4e+1-A_max.*P_wayp.*l6.*l7.*l29.*2.4e+1-J_min.*l4.*l7.*l29.*t7.*1.2e+1+J_max.*l4.*l6.*l29.*t7.*1.2e+1+A_max.*l6.*l8.*l29.*t7.^3.*4.0-l4.*l18.*l19.*l28.*l30.*l31.*1.2e+1-l4.*l18.*l20.*l27.*l30.*l31.*1.2e+1-A_max.*l6.*l8.*l23.^2.*t7.*(V_min-V_max).*2.4e+1+A_max.*J_min.*J_max.*l29.*l32.*t7.*1.2e+1-A_max.*J_max.*V_init.*l6.*l21.*l29.*2.4e+1-A_max.*J_max.*l2.*l6.*l29.*t7.*1.2e+1+A_max.*V_init.*l6.*l7.*l29.*t7.*2.4e+1+A_max.*J_max.^(7.0./2.0).*l18.*l24.^(3.0./2.0).*l28.*l31.^3.*8.0+A_max.*l18.*l20.*l27.^5.*l30.*l31.*t7.^2.*1.2e+1-A_max.*V_init.*l18.*l20.*l28.*l30.*l31.*2.4e+1+A_max.*l2.*l18.*l19.*l28.*l30.*l31.*1.2e+1+A_max.*l18.*l19.*l27.*l30.*l31.*l32.*1.2e+1)./(J_min.*l4.*l7.*l29.*-1.2e+1+J_max.*l4.*l6.*l29.*1.2e+1+A_max.*J_min.*J_max.*l29.*l32.*1.2e+1-A_max.*J_max.*l2.*l6.*l29.*1.2e+1+A_max.*V_init.*l6.*l7.*l29.*2.4e+1);

l2 = A_init.^2;
l3 = A_max.^2;
l4 = J_max.*V_init.*2.0;
l5 = J_max.*V_max.*2.0;
l6 = 1.0./A_max;
l7 = 1.0./J_min;
l8 = 1.0./J_max;
l9 = -l4;
l10 = -l3;
l11 = J_max.*l3.*l7;
l12 = l2+l5+l9+l10+l11;
l13 = (l6.*l8.*l12)./2.0;
t2 = [l13;l13;l13];

l2 = A_max.^2;
t5 = sqrt((J_min-J_max).*(J_min.*l2-J_max.*l2-A_init.^2.*J_min+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_min.*2.0+A_max.*J_min.*J_max.*t2.*2.0))./(J_min.^2-J_min.*J_max);

l2 = 1.0./J_min;
l3 = A_max.*l2;
l4 = -l3;
t3 = [l4;l4;l4];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t6 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
