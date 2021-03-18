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

function [t] = abcdeg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 13:55:05
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l9 = J_min.^2;
l10 = J_max.^2;
l11 = J_max.^3;
l13 = J_max.^5;
l14 = A_wayp.*J_min.*J_max.*2.0;
l15 = J_min.*J_max.*V_init.*2.0;
l16 = J_min.*J_max.*V_wayp.*2.0;
l22 = sqrt(3.0);
l23 = sqrt(6.0);
l6 = l4.^2;
l12 = l10.^2;
l17 = J_max.*l2;
l18 = J_min.*l4;
l19 = J_max.*l4;
l20 = J_max.*l7;
l21 = J_min.*l10;
l28 = A_min.*l13.*4.0;
l29 = A_wayp.*l10.*2.0;
l34 = A_init.*J_min.*l11.*6.0;
l35 = A_min.*J_min.*l11.*3.0;
l36 = J_min.*J_max.*l5.*3.0;
l38 = A_init.*A_wayp.*l11.*1.2e+1;
l42 = l3.*l10.*6.0;
l43 = l5.*l9.*3.0;
l45 = l8.*l10.*2.0;
l50 = A_min.*A_wayp.*J_max.*l9.*6.0;
l51 = A_min.*J_max.*V_init.*l9.*6.0;
l52 = A_min.*J_max.*V_wayp.*l9.*6.0;
l53 = T.*l9.*l11.*6.0;
l56 = A_init.*J_min.*J_max.*l5.*1.2e+1;
l60 = A_min.*A_wayp.*J_min.*l11.*2.4e+1;
l61 = A_wayp.*J_min.*T.*l11.*1.2e+1;
l65 = A_min.*l3.*l10.*4.0;
l66 = A_init.*l7.*l10.*6.0;
l67 = A_min.*l9.*l10.*3.0;
l68 = A_wayp.*l9.*l10.*6.0;
l70 = P_init.*l9.*l10.*1.2e+1;
l77 = A_min.*l7.*l11.*1.2e+1;
l78 = A_min.*l9.*l11.*8.0;
l79 = A_min.*P_wayp.*l9.*l10.*2.4e+1;
l80 = J_max.*T.*l5.*l9.*1.2e+1;
l82 = A_wayp.*T.*l9.*l10.*1.2e+1;
l83 = T.*V_wayp.*l9.*l10.*1.2e+1;
l27 = A_init.*l12.*6.0;
l30 = A_wayp.*l12.*6.0;
l37 = J_min.*T.*l12.*6.0;
l39 = -l35;
l40 = A_min.*J_min.*l12.*1.2e+1;
l44 = l6.*l9.*4.0;
l46 = A_min.*J_min.*l17.*3.0;
l47 = A_init.*J_max.*l18.*6.0;
l48 = A_min.*J_min.*l20.*3.0;
l49 = A_min.*A_wayp.*l21.*6.0;
l55 = l6.*l10.*8.0;
l57 = A_init.*A_wayp.*l21.*1.2e+1;
l59 = -l50;
l62 = A_init.*V_init.*l21.*1.2e+1;
l63 = -l51;
l64 = A_init.*V_wayp.*l21.*1.2e+1;
l69 = l7.*l21.*6.0;
l71 = T.*l2.*l21.*6.0;
l72 = T.*l9.*l19.*6.0;
l76 = -l66;
l81 = T.*l7.*l21.*-6.0;
l84 = -l82;
l85 = A_min.*T.*l2.*l21.*1.2e+1;
l88 = 1.0./(l11-l21).^2;
l91 = (l14-l29).^2;
l94 = l19.*(l11-l21).*-1.2e+1;
l95 = l19.*(l14-l29).*-1.2e+1;
l96 = (l11-l21).*(l14-l29).*6.0;
l98 = (l15-l16-l17-l18+l19+l20).^2;
l100 = l19.*(l15-l16-l17-l18+l19+l20).*-1.2e+1;
l101 = (l11-l21).*(l15-l16-l17-l18+l19+l20).*6.0;
l102 = (l14-l29).*(l15-l16-l17-l18+l19+l20).*6.0;
l31 = -l27;
l41 = -l37;
l54 = -l40;
l58 = -l48;
l74 = -l57;
l75 = -l62;
l89 = l88.^2;
l90 = l88.^3;
l93 = l91.*3.0;
l99 = l98.*3.0;
l92 = l89.^2;
l103 = l28+l54+l78+l96;
l107 = l30+l31+l34+l39+l41+l53+l67+l68;
l108 = l38+l49+l59+l61+l69+l74+l84;
l121 = l36+l42+l43+l45+l46+l47+l52+l58+l63+l64+l70+l71+l72+l75+l76+l81+l83;
l104 = l103.^2;
l105 = l103.^3;
l109 = A_min.*l107.*2.0;
l111 = A_min.*l108.*2.0;
l113 = (l88.*l103)./1.2e+1;
l122 = A_min.*l121.*2.0;
l106 = l104.^2;
l110 = -l109;
l112 = -l111;
l114 = -l113;
l115 = (l89.*l104)./2.4e+1;
l116 = (l90.*l105)./2.16e+2;
l123 = -l122;
l117 = (l92.*l106)./5.76e+2;
l118 = (l92.*l106)./6.912e+3;
l120 = l77+l95+l102+l112;
l124 = l60+l93+l94+l101+l110;
l144 = l44+l55+l56+l65+l79+l80+l85+l99+l100+l123;
l119 = -l117;
l125 = (l88.*l120)./3.0;
l126 = (l88.*l124)./3.0;
l128 = (l89.*l103.*l120)./3.0;
l130 = (l89.*l103.*l120)./3.6e+1;
l131 = (l89.*l103.*l124)./1.8e+1;
l133 = (l90.*l104.*l124)./3.6e+1;
l134 = (l90.*l104.*l124)./4.32e+2;
l145 = l88.*l144.*4.0;
l146 = (l88.*l144)./3.0;
l127 = -l126;
l129 = -l128;
l132 = -l131;
l135 = -l134;
l147 = -l146;
l136 = l115+l127;
l148 = l116+l125+l132;
l156 = l118+l130+l135+l147;
l137 = l136.^2;
l138 = l136.^3;
l149 = l148.^2;
l157 = l156.^2;
l158 = l156.^3;
l160 = l136.*l156.*(4.0./3.0);
l161 = l136.*l156.*7.2e+1;
l139 = l137.^2;
l140 = l138.*2.0;
l142 = l138./2.7e+1;
l150 = l149.^2;
l151 = l149.*2.7e+1;
l153 = l149./2.0;
l154 = l138.*l149.*4.0;
l159 = l158.*2.56e+2;
l162 = -l160;
l163 = -l161;
l165 = l137.*l157.*1.28e+2;
l166 = l136.*l149.*l156.*1.44e+2;
l141 = -l140;
l143 = -l142;
l152 = l150.*2.7e+1;
l155 = -l154;
l164 = l139.*l156.*1.6e+1;
l167 = -l166;
l168 = l152+l155+l159+l164+l165+l167;
l169 = sqrt(complex(l168));
l170 = l22.*l169.*3.0;
l171 = (l22.*l169)./1.8e+1;
l172 = l141+l151+l163+l170;
l174 = l143+l153+l162+l171;
l173 = sqrt(complex(l172));
l175 = l174.^(1.0./3.0);
l177 = 1.0./l174.^(1.0./6.0);
l176 = l175.^2;
l179 = l136.*l175.*6.0;
l180 = l23.*l148.*l173.*3.0;
l178 = l176.*9.0;
l181 = -l180;
l182 = l119+l129+l133+l137+l145+l178+l179;
l183 = sqrt(complex(l182));
l184 = 1.0./l182.^(1.0./4.0);
l185 = l137.*l183;
l187 = l156.*l183.*1.2e+1;
l189 = l176.*l183.*-9.0;
l190 = (l177.*l183)./6.0;
l192 = l136.*l175.*l183.*1.2e+1;
l186 = -l185;
l191 = -l190;
l193 = l180+l186+l187+l189+l192;
l194 = l181+l186+l187+l189+l192;
l195 = sqrt(complex(l193));
l196 = sqrt(complex(l194));
l197 = (l177.*l184.*l195)./6.0;
l198 = (l177.*l184.*l196)./6.0;
t7 = [l114+l191-l197;l114+l191+l197;l114+l190-l198;l114+l190+l198];

l2 = J_max.*t7;
l3 = A_init.^2;
l4 = A_min.^2;
l5 = -l2;
l6 = A_wayp+l5;
t2 = (J_min.*l4+J_max.*l3.*2.0-J_max.*(l3+l4+J_min.*V_init.*2.0+l6.^2+J_min.*l2.*t7+J_min.*l6.*t7.*2.0)+J_min.*J_max.*V_wayp.*2.0)./(A_min.*J_min.*J_max.*2.0);

l2 = 1.0./J_min;
t4 = (l2.*(A_init.*J_max+A_min.*J_min-A_min.*J_max+J_min.*J_max.*T-J_min.*J_max.*(t2+t7+l2.*(A_wayp-J_max.*t7))))./J_max;

t5 = (A_wayp-J_max.*t7)./J_min;

l2 = 1.0./J_max;
l3 = A_min.*l2;
l4 = -l3;
t3 = [l4;l4;l4;l4];

l2 = -A_min;
l3 = 1.0./J_min;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t6 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


