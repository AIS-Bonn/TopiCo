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

function [t] = abcdefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 13:55:05
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l10 = A_wayp.^2;
l11 = A_wayp.^3;
l13 = J_min.^2;
l14 = J_max.^2;
l15 = J_max.^3;
l17 = J_max.^5;
l19 = V_init.^2;
l20 = V_max.^2;
l21 = V_wayp.^2;
l22 = sqrt(3.0);
l23 = sqrt(6.0);
l4 = l2.^2;
l7 = l5.^2;
l9 = l8.^2;
l12 = l10.^2;
l16 = l14.^2;
l18 = l14.^3;
l27 = A_max.*A_wayp.*l6.*l17.*4.0;
l42 = A_max.*l6.*l11.*l15.*4.0;
l48 = A_max.*A_wayp.*l6.*l13.*l15.*4.0;
l50 = A_init.*A_max.*A_wayp.*J_max.*V_init.*l6.*l13.*2.4e+1;
l51 = A_init.*A_max.*A_wayp.*J_max.*V_max.*l6.*l13.*2.4e+1;
l52 = A_max.*A_wayp.*l3.*l6.*l13.*8.0;
l53 = A_max.*J_max.*l3.*l6.*l13.*8.0;
l54 = A_max.*P_init.*l6.*l13.*l15.*2.4e+1;
l55 = A_max.*P_wayp.*l6.*l13.*l15.*2.4e+1;
l58 = A_max.*V_max.*l6.*l13.*l15.*1.2e+1;
l59 = A_max.*V_wayp.*l6.*l13.*l15.*1.2e+1;
l64 = A_max.*A_wayp.*P_init.*l6.*l13.*l14.*2.4e+1;
l65 = A_max.*A_wayp.*P_wayp.*l6.*l13.*l14.*2.4e+1;
l66 = A_init.*A_max.*V_init.*l6.*l13.*l14.*2.4e+1;
l67 = A_init.*A_max.*V_max.*l6.*l13.*l14.*2.4e+1;
l68 = A_max.*T.*V_max.*l6.*l13.*l15.*2.4e+1;
l69 = A_max.*V_max.*V_wayp.*l6.*l13.*l14.*2.4e+1;
l79 = V_init.*l5.*l8.*l13.*l15.*1.2e+1;
l80 = V_max.*l5.*l8.*l13.*l15.*1.2e+1;
l81 = A_max.*l6.*l13.*l14.*l20.*1.2e+1;
l82 = A_max.*l6.*l13.*l14.*l21.*1.2e+1;
l83 = A_max.*A_wayp.*T.*V_max.*l6.*l13.*l14.*2.4e+1;
l85 = A_wayp.*J_max.*l2.*l5.*l8.*l13.*1.2e+1;
l86 = J_max.*V_init.*l5.*l8.*l10.*l13.*1.2e+1;
l87 = A_wayp.*V_init.*l5.*l8.*l13.*l14.*2.4e+1;
l88 = J_max.*V_max.*l5.*l8.*l10.*l13.*1.2e+1;
l89 = A_wayp.*V_max.*l5.*l8.*l13.*l14.*2.4e+1;
l90 = l2.*l5.*l8.*l10.*l13.*6.0;
l91 = l2.*l5.*l8.*l13.*l14.*6.0;
l24 = l9.*l18;
l25 = A_max.*l6.*l18;
l26 = A_wayp.*l9.*l17.*4.0;
l28 = l9.*l12.*l13;
l29 = l9.*l12.*l14;
l30 = l9.*l13.*l16;
l32 = -l27;
l33 = l4.*l7.*l13.*3.0;
l34 = l9.*l11.*l15.*4.0;
l35 = l9.*l10.*l16.*6.0;
l36 = A_max.*l6.*l12.*l14;
l37 = A_max.*l6.*l13.*l16;
l43 = A_max.*l6.*l10.*l16.*6.0;
l44 = A_wayp.*l9.*l13.*l15.*4.0;
l45 = J_max.*l9.*l11.*l13.*4.0;
l49 = V_init.*V_max.*l7.*l13.*l14.*2.4e+1;
l56 = J_max.*V_init.*l2.*l7.*l13.*1.2e+1;
l57 = J_max.*V_max.*l2.*l7.*l13.*1.2e+1;
l60 = l9.*l10.*l13.*l14.*6.0;
l61 = l7.*l13.*l14.*l19.*1.2e+1;
l62 = l7.*l13.*l14.*l20.*1.2e+1;
l63 = -l50;
l70 = -l55;
l72 = -l58;
l76 = -l65;
l77 = -l66;
l78 = -l69;
l84 = -l79;
l92 = -l85;
l93 = -l88;
l94 = -l89;
l95 = -l90;
l31 = -l25;
l38 = -l33;
l39 = -l29;
l40 = -l34;
l41 = -l30;
l46 = -l43;
l47 = -l44;
l71 = -l57;
l73 = -l60;
l74 = -l61;
l75 = -l62;
l96 = l24+l31+l37+l41;
l101 = l26+l32+l47+l48;
l113 = l35+l46+l59+l72+l73+l80+l84+l91;
l115 = l40+l42+l45+l53+l54+l67+l68+l70+l77+l87+l92+l94;
l130 = l28+l36+l38+l39+l49+l51+l52+l56+l63+l64+l71+l74+l75+l76+l78+l81+l82+l83+l86+l93+l95;
l97 = 1.0./l96;
l102 = l101.^2;
l103 = l101.^3;
l98 = l97.^2;
l99 = l97.^3;
l104 = l102.^2;
l105 = (l97.*l101)./4.0;
l114 = l97.*l113;
l120 = l97.*l115;
l131 = l97.*l130;
l100 = l98.^2;
l106 = l98.*l102.*(3.0./8.0);
l107 = (l99.*l103)./8.0;
l116 = (l98.*l101.*l113)./2.0;
l117 = l99.*l102.*l113.*(3.0./4.0);
l118 = (l99.*l102.*l113)./1.6e+1;
l127 = l98.*l101.*l115.*3.0;
l128 = (l98.*l101.*l115)./4.0;
l132 = l131.*1.2e+1;
l109 = -l107;
l110 = l100.*l104.*(3.0./2.56e+2);
l111 = l100.*l104.*(9.0./6.4e+1);
l119 = -l118;
l122 = (l106-l114).^2;
l125 = (l106-l114).^3.*-2.0;
l126 = (l106-l114).^3.*(-1.0./2.7e+1);
l129 = -l128;
l133 = -l132;
l112 = -l111;
l124 = l122.^2;
l134 = l109+l116+l120;
l141 = l110+l119+l129+l131;
l135 = l134.^2;
l142 = l141.^2;
l143 = l141.^3;
l145 = l141.*(l106-l114).*(-4.0./3.0);
l146 = l141.*(l106-l114).*-7.2e+1;
l147 = l124.*l141.*1.6e+1;
l136 = l135.^2;
l137 = l135.*2.7e+1;
l139 = l135./2.0;
l140 = l135.*(l106-l114).^3.*-4.0;
l144 = l143.*2.56e+2;
l148 = l122.*l142.*1.28e+2;
l149 = l135.*l141.*(l106-l114).*-1.44e+2;
l138 = l136.*2.7e+1;
l150 = l138+l140+l144+l147+l148+l149;
l151 = sqrt(complex(l150));
l152 = l22.*l151.*3.0;
l153 = (l22.*l151)./1.8e+1;
l154 = l125+l137+l146+l152;
l156 = l126+l139+l145+l153;
l155 = sqrt(complex(l154));
l157 = l156.^(1.0./3.0);
l159 = 1.0./l156.^(1.0./6.0);
l158 = l157.^2;
l162 = l157.*(l106-l114).*6.0;
l163 = l23.*l134.*l155.*3.0;
l160 = l158.*9.0;
l164 = -l163;
l165 = l112+l117+l122+l127+l133+l160+l162;
l166 = sqrt(complex(l165));
l167 = 1.0./l165.^(1.0./4.0);
l168 = l122.*l166;
l170 = l141.*l166.*1.2e+1;
l172 = l158.*l166.*-9.0;
l173 = (l159.*l166)./6.0;
l176 = l157.*l166.*(l106-l114).*1.2e+1;
l169 = -l168;
l174 = -l173;
l177 = l163+l169+l170+l172+l176;
l178 = l164+l169+l170+l172+l176;
l179 = sqrt(complex(l177));
l180 = sqrt(complex(l178));
l181 = (l159.*l167.*l179)./6.0;
l182 = (l159.*l167.*l180)./6.0;
t7 = [l105+l174-l181;l105+l174+l181;l105+l173-l182;l105+l173+l182];

l2 = J_max.*t7;
l3 = A_init.^2;
l4 = A_min.^2;
l5 = A_max.^2;
l6 = A_wayp.^2;
l7 = A_wayp.^3;
l13 = -l2;
l15 = J_max.*l5.*l7;
l16 = A_init.*A_min.*A_max.*J_min.*l6.*2.0;
l17 = A_wayp.*J_min.*J_max.*V_init.*l4.*2.0;
l21 = A_min.*A_max.*J_max.*l2.^3.*2.0;
l22 = A_min.*A_max.*J_min.*J_max.*T.*l6.*2.0;
l23 = J_min.*l2.*l3.*l4;
l27 = A_min.*A_max.*J_max.*l2.*l6.*2.0;
l28 = A_wayp.*J_min.*J_max.*l2.*l4.*2.0;
l29 = A_wayp.*J_max.*l2.^2.*l5;
l30 = J_min.*l2.^3.*l5;
l31 = A_min.*A_max.*J_min.*J_max.*T.*l2.^2.*2.0;
l35 = A_init.*A_min.*A_max.*J_min.*l2.^2.*2.0;
l14 = A_wayp+l13;
l18 = -l16;
l20 = -l17;
l25 = -l15;
l26 = -l22;
l32 = -l27;
l34 = J_min.*l5.*l6.*l13;
l19 = 1.0./l14;
t6 = (l19.*(l18+l20+l21+l23+l25+l26+l29+l30+l31+l32+l34+l35+J_min.*l2.^3.*l4-J_max.*l2.*l4.*l6.*2.0+A_wayp.*J_max.*l2.^2.*l4.*2.0+J_min.*J_max.*V_wayp.*l2.*l4.*2.0+A_wayp.*J_min.*l2.*l4.*l13+A_wayp.*J_min.*J_max.*V_max.*l4.*2.0-A_wayp.*J_min.*J_max.*V_wayp.*l4.*2.0)-l19.*(l18+l20+l21+l23+l25+l26+l29+l30+l31+l32+l34+l35-J_max.*l4.*l7+J_max.*l2.^3.*l4-J_min.*l2.*l4.*l6.*2.0+J_max.*l2.*l4.*l6+A_wayp.*J_min.*l2.^2.*l4.*2.0+J_min.*J_max.*V_max.*l2.*l4.*2.0+A_wayp.*J_max.*l2.*l4.*l13))./(l19.*(l28-J_min.*J_max.*l4.*l6.*2.0)+l19.*(l28-J_min.*J_max.*l2.^2.*l4.*2.0));

l2 = A_min.^2;
l3 = A_max.^2;
l4 = A_wayp.^2;
l5 = J_max.^2;
l6 = J_max.^3;
l7 = t7.^2;
t4 = (J_min.*l3.*l4-J_max.*l3.*l4-l3.*l6.*l7+A_init.^2.*J_min.*l2+J_min.*l3.*l5.*l7+A_min.*A_max.*J_max.*l4.*2.0-J_min.*J_max.*V_init.*l2.*2.0+J_min.*J_max.*V_max.*l2.*2.0+A_min.*A_max.*l6.*l7.*2.0+A_wayp.*l3.*l5.*t7.*2.0-A_min.*A_max.*A_wayp.*l5.*t7.*4.0-A_min.*A_max.*J_min.*l5.*l7.*2.0-A_wayp.*J_min.*J_max.*l3.*t7.*2.0-A_init.*A_min.*A_max.*A_wayp.*J_min.*2.0-A_min.*A_max.*A_wayp.*J_min.*J_max.*T.*2.0+A_init.*A_min.*A_max.*J_min.*J_max.*t7.*2.0+A_min.*A_max.*A_wayp.*J_min.*J_max.*t6.*2.0+A_min.*A_max.*A_wayp.*J_min.*J_max.*t7.*2.0+A_min.*A_max.*J_min.*T.*l5.*t7.*2.0-A_min.*A_max.*J_min.*l5.*t6.*t7.*2.0)./(A_min.*A_max.*J_min.*l5.*t7.*2.0-A_min.*A_max.*A_wayp.*J_min.*J_max.*2.0);

l2 = J_max.*t7;
l3 = 1.0./J_min;
l4 = -l2;
l5 = A_wayp+l4;
t2 = (l3.*(A_min.*J_min.*(A_init-J_max.*(t4+t6+t7+l3.*l5)+J_max.*T)-A_max.*l5.*(J_min-J_max)))./(A_min.*J_max);

l2 = -J_max;
t3 = -(A_init+l2.*(t2+t4+t6+t7+(A_wayp+l2.*t7)./J_min)+J_max.*T)./(J_min+l2);

t5 = (A_wayp-J_max.*t7)./J_min;

t1 = -(A_init+J_min.*t3)./J_max;

t = [t1, t2, t3, t4, t5, t6, t7];

end


