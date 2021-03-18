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

function [t] = abcdeg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,A_max,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 11:03:57
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_max.^2;
l5 = A_max.^3;
l7 = J_min.^2;
l8 = J_min.^3;
l10 = J_max.^2;
l11 = J_min.^5;
l12 = J_max.^3;
l14 = J_max.^5;
l15 = -A_max;
l16 = 1.0./A_max;
l18 = -J_max;
l19 = A_init.*A_max.*J_min.*2.0;
l20 = A_max.*A_wayp.*J_min.*2.0;
l21 = A_max.*A_wayp.*J_max.*2.0;
l22 = J_min.*J_max.*V_init.*2.0;
l23 = J_min.*J_max.*V_max.*2.0;
l27 = sqrt(3.0);
l32 = A_max.*J_min.*J_max.*T.*2.0;
l6 = l4.^2;
l9 = l7.^2;
l13 = l10.^2;
l17 = 1.0./l4;
l24 = J_min.*l2;
l25 = J_min.*l4;
l26 = J_max.*l4;
l28 = -l19;
l29 = A_init+l15;
l30 = -l22;
l31 = J_min+l18;
l34 = l4.*l18;
l35 = -l32;
l39 = A_max.*l8.*l14.*4.0;
l40 = A_max.*l11.*l12.*8.0;
l33 = -l25;
l36 = l29.^3;
l37 = l31.^2;
l38 = l31.^3;
l41 = A_max.*l9.*l13.*1.2e+1;
l94 = l20+l23+l24+l25+l28+l30+l34+l35;
l95 = l21+l23+l24+l25+l28+l30+l34+l35;
l42 = -l41;
l43 = J_min.*J_max.*l6.*l38.*1.2e+1;
l45 = l6.*l10.*l38.*8.0;
l46 = A_init.*l5.*l7.*l38.*1.2e+1;
l47 = A_max.*l3.*l7.*l38.*1.2e+1;
l48 = J_min.*l5.*l10.*l38.*1.2e+1;
l49 = J_min.*l5.*l12.*l37.*1.2e+1;
l50 = J_max.*l5.*l7.*l38.*1.2e+1;
l51 = J_max.*l5.*l8.*l37.*1.2e+1;
l55 = A_max.*J_max.*l2.*l7.*l38.*1.2e+1;
l56 = A_max.*J_max.*l2.*l8.*l37.*1.2e+1;
l57 = l24.*l26.*l38.*1.2e+1;
l58 = A_max.*P_init.*l7.*l10.*l38.*2.4e+1;
l59 = A_max.*P_wayp.*l7.*l10.*l38.*2.4e+1;
l60 = A_max.*V_init.*l7.*l10.*l38.*2.4e+1;
l61 = A_max.*V_init.*l7.*l12.*l37.*2.4e+1;
l62 = A_max.*V_init.*l8.*l10.*l37.*2.4e+1;
l63 = V_init.*l10.*l25.*l38.*2.4e+1;
l64 = l5.*l7.*l10.*l37.*2.4e+1;
l69 = l2.*l4.*l7.*l38.*2.4e+1;
l70 = A_max.*l2.*l7.*l10.*l37.*1.2e+1;
l73 = A_max.*l7.*l36.*l38.*4.0;
l75 = A_max.*J_max.*V_init.*l7.*l29.*l38.*2.4e+1;
l77 = l23+l24+l26+l30+l33;
l96 = l94.^2;
l97 = l94.^3;
l98 = l95.^2;
l99 = l95.^3;
l100 = l7.*l13.*l94.*6.0;
l101 = l9.*l10.*l94.*6.0;
l102 = l8.*l12.*l95.*6.0;
l103 = l9.*l10.*l95.*6.0;
l105 = l8.*l12.*l94.*1.2e+1;
l113 = J_max.*l24.*l37.*l94.*6.0;
l114 = J_max.*l25.*l37.*l94.*6.0;
l115 = J_max.*l25.*l37.*l95.*6.0;
l116 = J_min.*V_init.*l10.*l37.*l94.*1.2e+1;
l117 = J_max.*V_init.*l7.*l37.*l95.*1.2e+1;
l118 = l2.*l7.*l37.*l95.*6.0;
l119 = l4.*l7.*l37.*l95.*6.0;
l120 = l4.*l10.*l37.*l94.*6.0;
l128 = J_max.*l8.*l16.*l94.*l95.*6.0;
l130 = l7.*l10.*l16.*l94.*l95.*6.0;
l78 = l77.^2;
l79 = l39+l40+l42;
l84 = A_max.*J_min.*J_max.*l38.*l77.*1.2e+1;
l85 = J_min.*J_max.*V_init.*l38.*l77.*1.2e+1;
l86 = l24.*l38.*l77.*6.0;
l87 = l25.*l38.*l77.*6.0;
l89 = l26.*l38.*l77.*1.2e+1;
l90 = A_max.*J_min.*l10.*l37.*l77.*1.2e+1;
l91 = A_max.*J_max.*l7.*l37.*l77.*1.2e+1;
l104 = -l102;
l106 = -l105;
l107 = J_min.*l12.*l16.*l96.*3.0;
l108 = (l7.*l17.*l99)./2.0;
l109 = (l10.*l17.*l97)./2.0;
l110 = l7.*l10.*l16.*l96.*3.0;
l125 = J_min.*l37.*l77.*l95.*6.0;
l126 = J_max.*l37.*l77.*l94.*6.0;
l129 = J_min.*J_max.*l17.*l95.*l96.*(3.0./2.0);
l132 = l7.*l17.*l94.*l98.*(3.0./2.0);
l80 = 1.0./l79;
l83 = l38.*l78.*3.0;
l134 = l100+l101+l103+l104+l106;
l81 = l80.^2;
l82 = l80.^3;
l135 = l134.^2;
l136 = l134.^3;
l137 = (l80.*l134)./3.0;
l150 = l80.*(l43-l45-l46-l47-l57-l58+l59+l63+l69+l73+l75-l83-l85+l86-l87+l89+l108-l109+l113-l114-l115-l116+l117-l118+l119+l120+l125-l126+l129-l132).*(-1.0./2.0);
l151 = (l80.*(l43-l45-l46-l47-l57-l58+l59+l63+l69+l73+l75-l83-l85+l86-l87+l89+l108-l109+l113-l114-l115-l116+l117-l118+l119+l120+l125-l126+l129-l132))./2.0;
l138 = -l137;
l139 = (l81.*l135)./9.0;
l140 = (l82.*l136)./2.7e+1;
l145 = l81.*l134.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130).*(-1.0./6.0);
l146 = (l81.*l134.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130))./6.0;
l142 = -l140;
l148 = -(l139+(l80.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130))./3.0).^3;
l152 = l140+l146+l150;
l153 = l152.^2;
l154 = l148+l153;
l155 = sqrt(complex(l154));
l156 = l142+l145+l151+l155;
l157 = l156.^(1.0./3.0);
l158 = 1.0./l157;
l159 = l157./2.0;
l160 = -l159;
l161 = -l158.*(l139+(l80.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130))./3.0);
l162 = l158.*(l139+(l80.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130))./3.0).*(-1.0./2.0);
l163 = l157+l161;
l164 = l27.*l163.*5.0e-1i;
t4 = [l138+l157+l158.*(l139+(l80.*(l48+l49-l50+l51+l55-l56-l60-l61+l62-l64+l70-l84-l90+l91-l107+l110-l128+l130))./3.0);l138+l160+l162-l164;l138+l160+l162+l164];

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

l2 = 1.0./J_max;
t5 = -(l2.*(A_init.*J_min-A_max.*J_min+A_max.*J_max+J_min.*J_max.*T-J_min.*J_max.*(t2+t4+A_wayp.*l2)))./(J_min.*(J_min.*l2-1.0));

t7 = (A_wayp-J_min.*t5)./J_max;

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


