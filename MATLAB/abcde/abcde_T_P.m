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

function [t] = abcde_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,~,A_max,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 15:43:38
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_init.^5;
l7 = A_max.^2;
l8 = A_max.^3;
l10 = A_max.^5;
l12 = J_min.^2;
l13 = J_min.^3;
l14 = J_max.^2;
l15 = J_max.^3;
l16 = T.^2;
l17 = T.^3;
l18 = V_init.^2;
l19 = V_init.^3;
l20 = V_max.^2;
l21 = V_max.^3;
l28 = sqrt(3.0);
l4 = l2.^2;
l6 = l2.^3;
l9 = l7.^2;
l11 = l7.^3;
l22 = 1.0./l8;
l24 = 1.0./l13;
l25 = 1.0./l15;
l26 = 1.0./l12.^3;
l27 = 1.0./l14.^3;
l38 = A_init.*l10.*l13.*6.0;
l39 = A_max.*l5.*l13.*6.0;
l42 = J_min.*l10.*l15.*6.0;
l43 = J_max.*l10.*l13.*6.0;
l44 = A_init.*J_min.*l10.*l14.*6.0;
l55 = A_init.*J_max.*l10.*l12.*1.2e+1;
l65 = l3.*l8.*l13.*2.0e+1;
l68 = l10.*l12.*l14.*1.2e+1;
l69 = l13.*l15.*l19.*8.0;
l70 = l13.*l15.*l21.*8.0;
l72 = A_init.*J_max.*V_init.*l8.*l13.*2.4e+1;
l73 = A_max.*J_max.*V_init.*l3.*l13.*2.4e+1;
l74 = A_init.*J_max.*V_max.*l8.*l13.*2.4e+1;
l75 = A_max.*J_max.*V_max.*l3.*l13.*2.4e+1;
l78 = J_max.*l3.*l8.*l12.*4.0;
l80 = A_max.*V_init.*V_max.*l13.*l15.*4.8e+1;
l91 = A_init.*l8.*l13.*l14.*2.4e+1;
l93 = J_max.*l3.*l7.*l13.*2.4e+1;
l94 = J_max.*l2.*l8.*l13.*3.6e+1;
l96 = P_init.*l8.*l12.*l15.*4.8e+1;
l97 = P_wayp.*l8.*l12.*l15.*4.8e+1;
l99 = T.*l8.*l13.*l15.*2.4e+1;
l103 = A_max.*l13.*l15.*l18.*2.4e+1;
l104 = V_init.*l7.*l13.*l15.*2.4e+1;
l105 = V_init.*l8.*l12.*l15.*2.4e+1;
l106 = V_init.*l8.*l13.*l14.*2.4e+1;
l108 = A_max.*l13.*l15.*l20.*2.4e+1;
l109 = V_max.*l7.*l13.*l15.*2.4e+1;
l110 = V_max.*l8.*l12.*l15.*2.4e+1;
l111 = V_max.*l8.*l13.*l14.*2.4e+1;
l112 = V_init.*l13.*l15.*l20.*2.4e+1;
l113 = V_max.*l13.*l15.*l18.*2.4e+1;
l114 = A_init.*A_max.*V_init.*V_max.*l13.*l14.*4.8e+1;
l116 = V_init.*V_max.*l2.*l13.*l14.*2.4e+1;
l117 = V_init.*V_max.*l7.*l12.*l15.*2.4e+1;
l118 = V_init.*V_max.*l7.*l13.*l14.*2.4e+1;
l121 = T.*l10.*l12.*l14.*-1.2e+1;
l132 = A_init.*T.*l8.*l13.*l14.*4.8e+1;
l133 = A_init.*A_max.*l13.*l14.*l18.*2.4e+1;
l134 = A_init.*V_init.*l8.*l12.*l14.*2.4e+1;
l135 = A_max.*V_init.*l2.*l13.*l14.*2.4e+1;
l136 = J_max.*V_init.*l2.*l7.*l13.*3.6e+1;
l137 = A_init.*V_init.*l7.*l13.*l14.*4.8e+1;
l138 = A_init.*A_max.*l13.*l14.*l20.*2.4e+1;
l139 = A_init.*V_max.*l8.*l12.*l14.*2.4e+1;
l140 = A_max.*V_max.*l2.*l13.*l14.*2.4e+1;
l141 = J_max.*V_max.*l2.*l7.*l13.*3.6e+1;
l142 = A_init.*V_max.*l7.*l13.*l14.*4.8e+1;
l143 = A_max.*T.*V_init.*V_max.*l13.*l15.*-4.8e+1;
l147 = T.*V_init.*l7.*l13.*l15.*4.8e+1;
l151 = T.*V_max.*l7.*l13.*l15.*4.8e+1;
l152 = l8.*l13.*l15.*l17.*8.0;
l155 = l8.*l13.*l15.*l16.*2.4e+1;
l156 = l2.*l13.*l14.*l18.*1.2e+1;
l157 = l2.*l13.*l14.*l20.*1.2e+1;
l158 = l7.*l12.*l15.*l18.*1.2e+1;
l159 = l7.*l13.*l14.*l18.*1.2e+1;
l160 = l7.*l12.*l15.*l20.*1.2e+1;
l161 = l7.*l13.*l14.*l20.*1.2e+1;
l166 = J_max.*T.*l3.*l7.*l13.*-2.4e+1;
l171 = l2.*l7.*l13.*l14.*1.2e+1;
l172 = l2.*l8.*l12.*l14.*1.2e+1;
l173 = T.*V_init.*l8.*l13.*l14.*-2.4e+1;
l182 = A_max.*T.*V_init.*l2.*l13.*l14.*-2.4e+1;
l183 = A_init.*T.*V_max.*l7.*l13.*l14.*-4.8e+1;
l185 = T.*l2.*l7.*l13.*l14.*2.4e+1;
l187 = V_init.*l2.*l7.*l12.*l14.*1.2e+1;
l188 = V_max.*l2.*l7.*l12.*l14.*1.2e+1;
l193 = T.*l2.*l8.*l12.*l14.*-1.2e+1;
l196 = V_max.*l7.*l13.*l15.*l16.*-2.4e+1;
l198 = l2.*l7.*l13.*l14.*l16.*-1.2e+1;
l23 = 1.0./l11;
l29 = l22.^3;
l30 = l24.^3;
l31 = l25.^3;
l32 = l6.*l13;
l33 = l11.*l13;
l34 = l11.*l15;
l40 = J_min.*l11.*l14.*3.0;
l41 = J_max.*l11.*l12.*5.0;
l45 = A_max.*J_max.*l4.*l13.*6.0;
l46 = T.*l42;
l47 = T.*l43;
l48 = J_max.*V_init.*l4.*l13.*6.0;
l49 = J_max.*V_max.*l4.*l13.*6.0;
l50 = J_min.*V_init.*l9.*l15.*6.0;
l51 = J_max.*V_init.*l9.*l13.*6.0;
l52 = J_min.*V_max.*l9.*l15.*6.0;
l53 = J_max.*V_max.*l9.*l13.*6.0;
l56 = A_init.*J_max.*l9.*l13.*2.4e+1;
l61 = -l55;
l63 = l2.*l9.*l13.*1.5e+1;
l64 = l4.*l7.*l13.*1.5e+1;
l66 = l9.*l12.*l15.*1.2e+1;
l67 = l9.*l13.*l14.*1.2e+1;
l76 = J_min.*l2.*l9.*l14.*3.0;
l77 = J_max.*l4.*l7.*l12.*3.0;
l79 = J_max.*l2.*l9.*l12.*6.0;
l84 = -l68;
l85 = -l70;
l86 = A_init.*J_max.*T.*l9.*l13.*-2.4e+1;
l87 = -l72;
l88 = -l73;
l92 = A_init.*l9.*l12.*l14.*2.4e+1;
l95 = -l80;
l100 = T.*l9.*l12.*l15.*2.4e+1;
l101 = T.*l9.*l13.*l14.*2.4e+1;
l102 = V_init.*l9.*l12.*l14.*1.2e+1;
l107 = V_max.*l9.*l12.*l14.*1.2e+1;
l119 = -l93;
l120 = -l97;
l123 = -l106;
l125 = -l109;
l126 = -l110;
l127 = -l113;
l128 = -l114;
l131 = T.*l94;
l144 = T.*l103;
l145 = T.*l105;
l148 = T.*l108;
l149 = T.*l110;
l150 = T.*l111;
l163 = T.*l137;
l164 = T.*l140;
l167 = -l134;
l168 = -l135;
l169 = -l141;
l170 = -l142;
l174 = -l151;
l175 = l9.*l13.*l14.*l16.*-1.2e+1;
l176 = -l156;
l177 = -l157;
l178 = -l158;
l179 = -l159;
l180 = -l160;
l181 = -l161;
l186 = l16.*l91;
l189 = l16.*l104;
l191 = -l171;
l192 = -l172;
l194 = -l185;
l195 = -l188;
l35 = -l32;
l36 = -l33;
l37 = -l34;
l54 = -l40;
l57 = -l49;
l58 = -l52;
l59 = -l53;
l60 = T.*l45;
l62 = -l56;
l81 = -l63;
l82 = -l64;
l83 = -l67;
l89 = -l76;
l90 = -l77;
l122 = -l101;
l124 = -l107;
l129 = T.*l92;
l153 = l16.*l66;
l199 = l66+l83+l91+l99+l104+l125+l191;
l203 = l42+l43+l45+l62+l84+l92+l94+l95+l100+l103+l105+l108+l111+l119+l122+l123+l126+l132+l137+l140+l147+l155+l168+l170+l174+l192+l194;
l206 = l35+l36+l37+l38+l39+l41+l44+l46+l47+l48+l50+l51+l54+l57+l58+l59+l60+l61+l65+l69+l74+l75+l78+l79+l81+l82+l85+l86+l87+l88+l89+l90+l96+l102+l112+l116+l117+l118+l120+l121+l124+l127+l128+l129+l131+l133+l136+l138+l139+l143+l144+l145+l148+l149+l150+l152+l153+l163+l164+l166+l167+l169+l173+l175+l176+l177+l178+l179+l180+l181+l182+l183+l186+l187+l189+l193+l195+l196+l198;
l200 = l199.^3;
l201 = (l22.*l24.*l25.*l199)./2.4e+1;
l204 = (l23.*l26.*l27.*l199.*l203)./1.92e+2;
l207 = (l22.*l24.*l25.*l206)./8.0;
l202 = (l29.*l30.*l31.*l200)./6.912e+3;
l205 = -l204;
l208 = l202+l205+l207;
l209 = l208.^(1.0./3.0);
l210 = l209./2.0;
l212 = l28.*l209.*5.0e-1i;
l211 = -l210;
t4 = [l201+l209;l201+l211-l212;l201+l211+l212];

l2 = A_max.^2;
t5 = (J_max.*l2-J_min.*(l2+J_max.*V_max.*2.0+A_init.^2+A_max.*J_max.*t4.*2.0)+A_init.*A_max.*J_min.*2.0+J_min.*J_max.*V_init.*2.0+A_max.*J_min.*J_max.*T.*2.0)./(A_max.*J_min.*J_max.*2.0);

t2 = (A_init.*J_min-A_max.*J_min+A_max.*J_max-J_min.*J_max.*(t4+t5)+J_min.*J_max.*T)./(J_min.*J_max);

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

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

