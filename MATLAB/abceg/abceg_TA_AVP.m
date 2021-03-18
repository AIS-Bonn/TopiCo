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

function [t] = abceg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 13:55:05
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_wayp.^2;
l6 = A_wayp.^3;
l8 = J_min.^2;
l9 = J_max.^2;
l10 = J_max.^3;
l11 = T.^2;
l12 = T.^3;
l14 = V_init.^2;
l15 = V_wayp.^2;
l22 = sqrt(3.0);
l23 = sqrt(6.0);
l30 = A_init.*A_wayp.*J_min.*J_max.*1.2e+1;
l31 = A_init.*J_min.*J_max.*P_init.*4.8e+1;
l32 = A_init.*J_min.*J_max.*P_wayp.*4.8e+1;
l33 = A_wayp.*J_min.*J_max.*P_init.*4.8e+1;
l34 = A_wayp.*J_min.*J_max.*P_wayp.*4.8e+1;
l43 = J_min.*J_max.*V_init.*V_wayp.*4.8e+1;
l66 = A_init.*J_min.*J_max.*T.*V_wayp.*4.8e+1;
l67 = A_wayp.*J_min.*J_max.*T.*V_init.*4.8e+1;
l4 = l2.^2;
l7 = l5.^2;
l13 = l11.^2;
l16 = 1.0./l8;
l18 = 1.0./l9;
l26 = A_init.*l6.*4.0;
l27 = A_wayp.*l3.*4.0;
l35 = A_init.*J_min.*l9.*4.0;
l36 = A_init.*J_max.*l8.*4.0;
l37 = J_min.*J_max.*l2.*6.0;
l38 = A_wayp.*J_min.*l9.*4.0;
l39 = A_wayp.*J_max.*l8.*4.0;
l40 = J_min.*J_max.*l5.*6.0;
l41 = J_max.*T.*l3.*4.0;
l42 = J_max.*T.*l6.*4.0;
l44 = J_min.*T.*l10.*4.0;
l45 = -l30;
l46 = -l32;
l47 = -l33;
l50 = A_init.*P_init.*l8.*2.4e+1;
l51 = A_init.*P_init.*l9.*2.4e+1;
l52 = A_init.*P_wayp.*l8.*2.4e+1;
l53 = A_wayp.*P_init.*l8.*2.4e+1;
l54 = A_init.*P_wayp.*l9.*2.4e+1;
l55 = A_wayp.*P_init.*l9.*2.4e+1;
l56 = A_wayp.*P_wayp.*l8.*2.4e+1;
l57 = A_wayp.*P_wayp.*l9.*2.4e+1;
l59 = l2.*l5.*6.0;
l60 = J_min.*J_max.*l14.*2.4e+1;
l61 = J_min.*J_max.*l15.*2.4e+1;
l62 = P_init.*T.*l10.*2.4e+1;
l63 = P_wayp.*T.*l10.*2.4e+1;
l64 = V_init.*V_wayp.*l8.*2.4e+1;
l65 = V_init.*V_wayp.*l9.*2.4e+1;
l68 = T.*l8.*l9.*4.0;
l70 = J_min.*l10.*l11.*6.0;
l80 = l8.*l14.*1.2e+1;
l81 = l9.*l14.*1.2e+1;
l82 = l8.*l15.*1.2e+1;
l83 = l9.*l15.*1.2e+1;
l84 = -l67;
l85 = A_init.*J_max.*T.*l5.*1.2e+1;
l86 = A_wayp.*J_max.*T.*l2.*1.2e+1;
l87 = A_init.*J_min.*T.*l9.*1.2e+1;
l88 = A_wayp.*J_min.*T.*l9.*1.2e+1;
l89 = A_init.*T.*V_wayp.*l8.*2.4e+1;
l90 = A_wayp.*T.*V_init.*l8.*2.4e+1;
l91 = A_init.*T.*V_wayp.*l9.*2.4e+1;
l92 = A_wayp.*T.*V_init.*l9.*2.4e+1;
l93 = J_max.*P_init.*T.*l8.*2.4e+1;
l94 = J_min.*P_init.*T.*l9.*4.8e+1;
l95 = J_max.*P_wayp.*T.*l8.*2.4e+1;
l96 = J_min.*P_wayp.*T.*l9.*4.8e+1;
l97 = V_init.*l10.*l11.*1.2e+1;
l98 = V_wayp.*l10.*l11.*1.2e+1;
l99 = A_init.*A_wayp.*J_min.*J_max.*l11.*2.4e+1;
l112 = A_init.*A_wayp.*l8.*l11.*1.2e+1;
l113 = A_init.*J_max.*l8.*l12.*-4.0;
l114 = A_init.*J_min.*l9.*l12.*8.0;
l115 = A_wayp.*J_min.*l9.*l12.*8.0;
l116 = J_max.*V_init.*l8.*l11.*1.2e+1;
l117 = J_min.*V_init.*l9.*l11.*2.4e+1;
l118 = J_max.*V_wayp.*l8.*l11.*1.2e+1;
l119 = J_min.*V_wayp.*l9.*l11.*2.4e+1;
l120 = l2.*l9.*l11.*6.0;
l121 = l5.*l9.*l11.*6.0;
l17 = l16.^2;
l19 = l16.^3;
l20 = l18.^2;
l21 = l18.^3;
l28 = -l26;
l29 = -l27;
l48 = -l38;
l49 = -l39;
l58 = -l42;
l69 = J_min.*l10.*l13.*2.0;
l71 = -l50;
l72 = -l51;
l73 = -l56;
l74 = -l57;
l75 = -l60;
l76 = -l61;
l77 = -l62;
l78 = -l64;
l79 = -l65;
l101 = l12.*l39;
l102 = l8.*l9.*l13;
l103 = -l86;
l104 = -l88;
l105 = -l89;
l106 = -l91;
l107 = -l93;
l108 = -l96;
l109 = -l97;
l110 = -l98;
l111 = -l99;
l122 = -l115;
l123 = -l116;
l124 = -l118;
l24 = l17.^2;
l25 = l20.^2;
l125 = -l102;
l126 = l35+l36+l44+l48+l49+l68;
l130 = l37+l40+l45+l70+l87+l104;
l127 = l126.^2;
l128 = l126.^3;
l131 = (l16.*l18.*l126)./1.2e+1;
l134 = (l16.*l18.*l130)./3.0;
l139 = (l17.*l20.*l126.*l130)./1.8e+1;
l160 = l4+l7+l28+l29+l31+l34+l41+l43+l46+l47+l52+l53+l54+l55+l58+l59+l63+l66+l69+l71+l72+l73+l74+l75+l76+l77+l78+l79+l80+l81+l82+l83+l84+l85+l90+l92+l94+l95+l101+l103+l105+l106+l107+l108+l109+l110+l111+l112+l113+l114+l117+l119+l120+l121+l122+l123+l124+l125;
l129 = l127.^2;
l132 = (l17.*l20.*l127)./2.4e+1;
l133 = (l19.*l21.*l128)./2.16e+2;
l135 = -l134;
l140 = -l139;
l141 = (l19.*l21.*l127.*l130)./3.6e+1;
l142 = (l19.*l21.*l127.*l130)./4.32e+2;
l161 = l16.*l18.*l160.*4.0;
l163 = (l16.*l18.*l160)./3.0;
l136 = (l24.*l25.*l129)./5.76e+2;
l137 = (l24.*l25.*l129)./6.912e+3;
l143 = -l142;
l144 = l132+l135;
l152 = l133+l140;
l162 = -l161;
l138 = -l136;
l145 = l144.^2;
l146 = l144.^3;
l153 = l152.^2;
l164 = l137+l143+l163;
l147 = l145.^2;
l148 = l146.*2.0;
l150 = l146./2.7e+1;
l154 = l153.^2;
l155 = l153.*2.7e+1;
l157 = l153./2.0;
l158 = l146.*l153.*4.0;
l165 = l164.^2;
l166 = l164.^3;
l168 = l144.*l164.*(4.0./3.0);
l169 = l144.*l164.*7.2e+1;
l174 = l144.*l153.*l164.*1.44e+2;
l149 = -l148;
l151 = -l150;
l156 = l154.*2.7e+1;
l159 = -l158;
l167 = l166.*2.56e+2;
l170 = l147.*l164.*1.6e+1;
l171 = -l168;
l172 = -l169;
l173 = l145.*l165.*1.28e+2;
l175 = -l174;
l176 = l156+l159+l167+l170+l173+l175;
l177 = sqrt(complex(l176));
l178 = l22.*l177.*3.0;
l179 = (l22.*l177)./1.8e+1;
l180 = l149+l155+l172+l178;
l182 = l151+l157+l171+l179;
l181 = sqrt(complex(l180));
l183 = l182.^(1.0./3.0);
l185 = 1.0./l182.^(1.0./6.0);
l184 = l183.^2;
l187 = l144.*l183.*6.0;
l188 = l23.*l152.*l181.*3.0;
l186 = l184.*9.0;
l189 = -l188;
l190 = l138+l141+l145+l162+l186+l187;
l191 = sqrt(complex(l190));
l192 = 1.0./l190.^(1.0./4.0);
l193 = l145.*l191;
l195 = l164.*l191.*1.2e+1;
l197 = l184.*l191.*-9.0;
l198 = (l185.*l191)./6.0;
l200 = l144.*l183.*l191.*1.2e+1;
l194 = -l193;
l199 = -l198;
l201 = l188+l194+l195+l197+l200;
l202 = l189+l194+l195+l197+l200;
l203 = sqrt(complex(l201));
l204 = sqrt(complex(l202));
l205 = (l185.*l192.*l203)./6.0;
l206 = (l185.*l192.*l204)./6.0;
t2 = [l131+l199-l206;l131+l199+l206;l131+l198-l205;l131+l198+l205];

l2 = J_max.*t2;
l3 = J_max.*T;
l4 = -A_wayp;
l5 = -J_max;
l6 = -l2;
l7 = J_min+l5;
l8 = l7.^2;
l9 = A_init+l3+l4+l6;
l10 = l9.^2;
t7 = (J_min.*l10+V_init.*l8.*2.0-V_wayp.*l8.*2.0+l5.*l10+A_init.*T.*l8.*2.0+T.*l3.*l8+l2.*l7.*l9.*2.0+l6.*l8.*t2)./(l2.*l8.*2.0+J_min.*l7.*l9.*2.0-J_max.*l7.*l9.*2.0);

l2 = -J_max;
t3 = -(A_init-A_wayp+l2.*t2+J_max.*T)./(J_min+l2);

t1 = T-t2-t3-t7;

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

