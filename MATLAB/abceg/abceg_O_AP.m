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

function [t] = abceg_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,A_max,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l9 = J_min.^2;
l10 = J_max.^2;
l11 = J_max.^3;
l13 = J_max.^5;
l15 = V_init.^2;
l16 = V_max.^2;
l18 = sqrt(3.0);
l19 = sqrt(6.0);
l4 = l2.^2;
l6 = l5.^2;
l12 = l10.^2;
l14 = l10.^3;
l20 = J_min.*l13.*6.0;
l21 = A_max.*l13.*8.0;
l22 = A_wayp.*l13.*1.2e+1;
l40 = A_max.*A_wayp.*J_min.*l11.*3.6e+1;
l41 = A_max.*J_min.*V_max.*l11.*2.4e+1;
l42 = A_wayp.*J_min.*V_max.*l11.*2.4e+1;
l43 = A_max.*l8.*l10.*4.0;
l44 = A_max.*l9.*l11.*4.0;
l45 = J_min.*l5.*l11.*6.0;
l46 = V_max.*l9.*l11.*1.2e+1;
l47 = A_init.*A_max.*J_max.*V_init.*l9.*2.4e+1;
l48 = A_max.*A_wayp.*J_min.*V_max.*l10.*2.4e+1;
l50 = A_max.*l3.*l9.*8.0;
l51 = A_max.*l7.*l11.*1.2e+1;
l52 = A_wayp.*l5.*l11.*1.2e+1;
l54 = A_wayp.*l9.*l11.*1.2e+1;
l55 = J_min.*l7.*l11.*2.4e+1;
l57 = A_max.*A_wayp.*l9.*l10.*1.2e+1;
l58 = A_max.*J_min.*l7.*l10.*1.2e+1;
l59 = A_wayp.*J_min.*l5.*l10.*1.2e+1;
l60 = A_max.*P_init.*l9.*l10.*2.4e+1;
l61 = A_max.*P_wayp.*l9.*l10.*2.4e+1;
l62 = J_max.*V_init.*l2.*l9.*1.2e+1;
l63 = J_max.*V_init.*l5.*l9.*1.2e+1;
l64 = J_min.*V_max.*l5.*l10.*1.2e+1;
l65 = A_max.*V_max.*l9.*l10.*2.4e+1;
l66 = A_wayp.*V_max.*l9.*l10.*2.4e+1;
l67 = l2.*l5.*l9.*6.0;
l77 = l7.*l9.*l10.*1.2e+1;
l78 = l9.*l10.*l15.*1.2e+1;
l79 = l9.*l10.*l16.*1.2e+1;
l17 = l14.*3.0;
l23 = -l20;
l24 = l6.*l9;
l25 = l6.*l10;
l26 = -l22;
l27 = A_max.*A_wayp.*l12.*2.4e+1;
l28 = A_max.*J_min.*l12.*1.2e+1;
l29 = A_wayp.*J_min.*l12.*2.4e+1;
l30 = J_min.*V_max.*l12.*1.2e+1;
l31 = l4.*l9.*3.0;
l32 = l5.*l12.*6.0;
l33 = l9.*l12.*3.0;
l39 = l7.*l12.*1.2e+1;
l56 = -l47;
l69 = -l54;
l73 = -l61;
l74 = -l64;
l76 = -l67;
l80 = -l78;
l35 = -l28;
l37 = -l31;
l38 = -l25;
l81 = l17+l23+l33;
l82 = 1.0./l81;
l86 = l21+l26+l29+l35+l44+l69;
l103 = l24+l37+l38+l43+l48+l50+l56+l60+l62+l63+l73+l74+l76+l79+l80;
l83 = l82.^2;
l84 = l82.^3;
l87 = l86.^2;
l88 = l86.^3;
l90 = (l82.*l86)./4.0;
l101 = -l82.*(l41-l42-l51+l52+l58-l59-l65+l66);
l112 = l82.*l103;
l85 = l83.^2;
l89 = l87.^2;
l91 = -l90;
l92 = l83.*l87.*(3.0./8.0);
l93 = (l84.*l88)./8.0;
l105 = l83.*l86.*(l41-l42-l51+l52+l58-l59-l65+l66).*3.0;
l106 = l83.*l86.*(l41-l42-l51+l52+l58-l59-l65+l66).*(-1.0./4.0);
l109 = (l83.*l86.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./2.0;
l110 = l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77).*(-3.0./4.0);
l113 = l112.*1.2e+1;
l95 = l85.*l89.*(3.0./2.56e+2);
l96 = l85.*l89.*(9.0./6.4e+1);
l115 = (l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).^2;
l118 = (l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).^3.*-2.0;
l119 = (l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).^3.*(-1.0./2.7e+1);
l120 = l93+l101+l109;
l99 = -l96;
l117 = l115.^2;
l121 = l120.^2;
l128 = (l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).^2;
l131 = (l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).^3.*2.56e+2;
l132 = (l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).*(l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).*(4.0./3.0);
l133 = (l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).*(l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).*7.2e+1;
l122 = l121.^2;
l123 = l121.*2.7e+1;
l125 = l121./2.0;
l126 = l121.*(l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).^3.*-4.0;
l134 = -l132;
l135 = -l133;
l137 = l117.*(l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).*1.6e+1;
l138 = l115.*l128.*1.28e+2;
l139 = l121.*(l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).*(l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).*1.44e+2;
l124 = l122.*2.7e+1;
l140 = -l139;
l141 = l124+l126+l131+l137+l138+l140;
l142 = sqrt(complex(l141));
l143 = l18.*l142.*3.0;
l144 = (l18.*l142)./1.8e+1;
l145 = l118+l123+l135+l143;
l147 = l119+l125+l134+l144;
l146 = sqrt(complex(l145));
l148 = l147.^(1.0./3.0);
l150 = 1.0./l147.^(1.0./6.0);
l149 = l148.^2;
l153 = l148.*(l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).*6.0;
l154 = l19.*l120.*l146.*3.0;
l151 = l149.*9.0;
l155 = -l154;
l156 = l99+l105+l110+l113+l115+l151+l153;
l157 = sqrt(complex(l156));
l158 = 1.0./l156.^(1.0./4.0);
l159 = l115.*l157;
l162 = l157.*(l95+l106-l112+(l84.*l87.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77))./1.6e+1).*1.2e+1;
l164 = l149.*l157.*-9.0;
l165 = (l150.*l157)./6.0;
l168 = l148.*l157.*(l92+l82.*(l27+l30-l32-l39-l40+l45-l46+l55+l57-l77)).*1.2e+1;
l160 = -l159;
l166 = -l165;
l169 = l154+l160+l162+l164+l168;
l170 = l155+l160+l162+l164+l168;
l171 = sqrt(complex(l169));
l172 = sqrt(complex(l170));
l173 = (l150.*l158.*l171)./6.0;
l174 = (l150.*l158.*l172)./6.0;
t7 = [l91+l166-l173;l91+l166+l173;l91+l165-l174;l91+l165+l174];

l2 = A_max.^2;
l3 = J_max.^2;
l4 = t7.^2;
t2 = (-J_min.*l2+J_max.*l2+A_init.^2.*J_min-J_max.^3.*l4-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_max.*2.0+A_wayp.*l3.*t7.*2.0+J_min.*l3.*l4-A_wayp.*J_min.*J_max.*t7.*2.0)./(A_max.*J_min.*J_max.*2.0);

t3 = -(A_max-A_wayp+J_max.*t7)./J_min;

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


