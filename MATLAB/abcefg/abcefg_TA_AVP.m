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

function [t] = abcefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Sep-2020 17:24:53
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_max.^2;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l10 = J_min.^2;
l11 = J_min.^3;
l13 = J_max.^2;
l14 = T.^2;
l15 = V_init.^2;
l16 = V_wayp.^2;
l17 = A_init.*A_min;
l18 = A_init.*A_max;
l19 = -A_max;
l22 = A_init.*A_wayp.*J_max.*6.0;
l23 = A_init.*J_min.*J_max.*6.0;
l24 = A_wayp.*J_min.*J_max.*6.0;
l25 = sqrt(3.0);
l26 = sqrt(6.0);
l29 = A_wayp.*J_max.*V_init.*1.2e+1;
l30 = A_wayp.*J_max.*V_wayp.*1.2e+1;
l44 = A_init.*A_wayp.*J_max.*T.*1.2e+1;
l60 = A_min.*A_max.*A_wayp.*J_min.*J_max.*V_init.*2.4e+1;
l61 = A_min.*A_max.*A_wayp.*J_min.*J_max.*V_wayp.*2.4e+1;
l4 = l2.^2;
l9 = l7.^2;
l12 = l10.^2;
l20 = l3.*2.0;
l21 = l8.*6.0;
l27 = -l18;
l28 = -l22;
l31 = A_min+l19;
l32 = A_init.*l7.*6.0;
l33 = A_wayp.*l2.*6.0;
l34 = J_max.*l2.*3.0;
l35 = J_max.*l7.*3.0;
l36 = J_min.*l13.*3.0;
l37 = J_max.*l10.*3.0;
l38 = V_init.*l13.*6.0;
l39 = V_wayp.*l13.*6.0;
l40 = -l30;
l42 = P_wayp.*l13.*1.2e+1;
l48 = A_init.*T.*l13.*6.0;
l58 = A_max.*J_min.*J_max.*V_init.*l17.*2.4e+1;
l59 = A_max.*J_min.*J_max.*V_wayp.*l17.*2.4e+1;
l65 = A_max.*l11.*l17.*1.2e+1;
l66 = A_min.*A_max.*J_min.*l3.*1.2e+1;
l67 = A_min.*A_max.*A_wayp.*l11.*1.2e+1;
l68 = A_min.*A_max.*J_min.*l8.*2.0e+1;
l69 = J_min.*l5.*l8.*4.0;
l70 = J_max.*l5.*l11.*6.0;
l71 = J_max.*l6.*l11.*6.0;
l73 = -l60;
l74 = A_max.*J_min.*l7.*l17.*1.2e+1;
l75 = A_min.*A_max.*A_wayp.*J_min.*l2.*1.2e+1;
l76 = A_max.*J_max.*l10.*l17.*1.2e+1;
l77 = A_min.*A_max.*J_min.*J_max.*l2.*1.2e+1;
l78 = A_min.*A_max.*J_min.*J_max.*l7.*1.2e+1;
l79 = A_min.*A_max.*A_wayp.*J_max.*l10.*3.6e+1;
l80 = A_min.*A_max.*J_max.*V_init.*l2.*2.4e+1;
l81 = A_min.*A_max.*J_max.*V_init.*l7.*2.4e+1;
l82 = A_min.*A_max.*J_max.*V_wayp.*l2.*2.4e+1;
l83 = A_min.*A_max.*J_max.*V_wayp.*l7.*2.4e+1;
l84 = A_min.*A_max.*J_min.*P_init.*l13.*4.8e+1;
l85 = A_min.*A_max.*J_max.*T.*l11.*1.2e+1;
l86 = A_init.*J_min.*J_max.*V_init.*l6.*2.4e+1;
l87 = A_min.*A_max.*J_min.*V_init.*l13.*2.4e+1;
l88 = A_init.*J_min.*J_max.*V_wayp.*l6.*2.4e+1;
l89 = A_wayp.*J_min.*J_max.*V_init.*l6.*2.4e+1;
l90 = A_min.*A_max.*J_min.*V_wayp.*l13.*2.4e+1;
l91 = A_wayp.*J_min.*J_max.*V_wayp.*l6.*2.4e+1;
l92 = J_min.*J_max.*l2.*l5.*6.0;
l93 = J_min.*J_max.*l2.*l6.*6.0;
l94 = J_min.*J_max.*l5.*l7.*6.0;
l95 = J_min.*J_max.*l6.*l7.*6.0;
l96 = A_min.*A_max.*V_init.*V_wayp.*l13.*4.8e+1;
l100 = A_init.*l6.*l11.*1.2e+1;
l101 = J_min.*l3.*l6.*1.2e+1;
l102 = J_min.*l6.*l8.*1.6e+1;
l108 = A_min.*A_max.*l2.*l7.*1.2e+1;
l113 = A_init.*J_min.*l6.*l7.*1.2e+1;
l114 = A_wayp.*J_min.*l2.*l6.*1.2e+1;
l115 = A_init.*A_wayp.*l6.*l10.*2.4e+1;
l118 = A_init.*J_max.*l6.*l10.*1.2e+1;
l119 = A_min.*A_max.*l10.*l13.*1.4e+1;
l120 = A_wayp.*J_max.*l5.*l10.*1.2e+1;
l121 = A_wayp.*J_max.*l6.*l10.*2.4e+1;
l122 = J_max.*V_init.*l2.*l5.*1.2e+1;
l123 = J_max.*V_init.*l2.*l6.*1.2e+1;
l124 = A_min.*A_max.*l13.*l15.*2.4e+1;
l125 = J_max.*V_wayp.*l2.*l5.*1.2e+1;
l126 = J_max.*V_init.*l5.*l7.*1.2e+1;
l127 = J_max.*V_wayp.*l2.*l6.*1.2e+1;
l128 = J_max.*V_init.*l6.*l7.*1.2e+1;
l129 = A_min.*A_max.*l13.*l16.*2.4e+1;
l130 = J_max.*V_wayp.*l5.*l7.*1.2e+1;
l131 = J_max.*V_wayp.*l6.*l7.*1.2e+1;
l132 = l2.*l5.*l7.*6.0;
l133 = l2.*l6.*l7.*6.0;
l134 = J_min.*P_init.*l5.*l13.*2.4e+1;
l135 = J_min.*P_init.*l6.*l13.*2.4e+1;
l136 = J_max.*T.*l6.*l11.*1.2e+1;
l137 = J_min.*V_init.*l5.*l13.*1.2e+1;
l138 = J_max.*V_init.*l5.*l10.*1.2e+1;
l139 = J_min.*V_init.*l6.*l13.*1.2e+1;
l140 = J_max.*V_init.*l6.*l10.*1.2e+1;
l141 = J_min.*V_wayp.*l5.*l13.*1.2e+1;
l142 = J_max.*V_wayp.*l5.*l10.*1.2e+1;
l143 = J_min.*V_wayp.*l6.*l13.*1.2e+1;
l144 = J_max.*V_wayp.*l6.*l10.*1.2e+1;
l145 = l2.*l5.*l10.*6.0;
l146 = l5.*l7.*l10.*6.0;
l147 = l6.*l7.*l10.*6.0;
l148 = l5.*l10.*l13.*7.0;
l149 = l6.*l10.*l13.*7.0;
l150 = V_init.*V_wayp.*l5.*l13.*2.4e+1;
l151 = V_init.*V_wayp.*l6.*l13.*2.4e+1;
l174 = l2.*l6.*l10.*1.8e+1;
l179 = l5.*l13.*l15.*1.2e+1;
l180 = l6.*l13.*l15.*1.2e+1;
l181 = l5.*l13.*l16.*1.2e+1;
l182 = l6.*l13.*l16.*1.2e+1;
l183 = A_min.*A_max.*J_min.*J_max.*T.*l2.*-1.2e+1;
l184 = A_min.*A_max.*J_min.*T.*V_init.*l13.*-2.4e+1;
l185 = A_min.*A_max.*J_min.*T.*V_wayp.*l13.*-2.4e+1;
l186 = J_min.*J_max.*T.*l2.*l6.*1.2e+1;
l187 = A_init.*J_max.*T.*l6.*l10.*2.4e+1;
l188 = A_min.*A_max.*T.*l10.*l13.*1.2e+1;
l189 = J_min.*J_max.*T.*l6.*l7.*1.2e+1;
l191 = J_min.*T.*V_init.*l5.*l13.*2.4e+1;
l192 = J_min.*T.*V_wayp.*l6.*l13.*2.4e+1;
l194 = A_wayp.*J_max.*T.*l6.*l10.*-2.4e+1;
l195 = T.*l6.*l10.*l13.*1.2e+1;
l197 = A_min.*A_max.*l10.*l13.*l14.*1.2e+1;
l41 = -l32;
l43 = -l39;
l45 = A_min.*A_max.*l4.*6.0;
l46 = A_min.*A_max.*l9.*6.0;
l47 = A_min.*A_max.*l12.*2.0;
l49 = l5.*l12;
l50 = l6.*l12;
l54 = l4.*l5.*3.0;
l55 = l4.*l6.*3.0;
l56 = l5.*l9.*3.0;
l57 = l6.*l9.*3.0;
l62 = l17+l27;
l63 = 1.0./l31;
l72 = -l59;
l97 = -l66;
l98 = -l67;
l99 = -l68;
l104 = -l78;
l105 = -l79;
l106 = -l81;
l107 = -l82;
l109 = -l84;
l110 = -l86;
l111 = -l87;
l112 = -l91;
l116 = -l92;
l117 = -l93;
l153 = T.*l78;
l157 = -l113;
l158 = -l114;
l159 = -l115;
l160 = -l118;
l161 = -l122;
l162 = -l123;
l163 = -l124;
l164 = -l129;
l165 = -l130;
l166 = -l131;
l167 = -l132;
l168 = -l133;
l169 = -l140;
l170 = -l141;
l171 = -l142;
l172 = -l143;
l173 = -l145;
l177 = -l150;
l178 = -l151;
l193 = -l189;
l196 = -l195;
l51 = -l45;
l52 = -l46;
l64 = l63.^2;
l156 = l62.^2;
l198 = J_min.*J_max.*l18.*l63.*6.0;
l200 = A_wayp.*J_min.*l18.*l63.*1.2e+1;
l201 = A_max.*A_wayp.*J_min.*J_max.*l63.*-6.0;
l202 = A_max.*J_min.*l2.*l63.*6.0;
l203 = A_max.*J_min.*l7.*l63.*6.0;
l204 = A_max.*J_max.*l10.*l63.*6.0;
l206 = A_max.*A_wayp.*J_min.*J_max.*T.*l63.*1.2e+1;
l207 = A_max.*J_min.*T.*l13.*l63.*6.0;
l210 = A_init.*J_max.*l62.*l63.*6.0;
l211 = A_wayp.*J_max.*l62.*l63.*6.0;
l212 = J_min.*J_max.*l62.*l63.*6.0;
l213 = A_init.*A_wayp.*l62.*l63.*1.2e+1;
l216 = l2.*l62.*l63.*6.0;
l218 = l7.*l62.*l63.*6.0;
l219 = A_wayp.*J_max.*T.*l62.*l63.*1.2e+1;
l220 = T.*l13.*l62.*l63.*6.0;
l242 = l65+l76+l85+l98+l100+l105+l120+l121+l136+l160+l188+l196;
l253 = l77+l90+l94+l95+l104+l111+l116+l117+l137+l138+l139+l144+l146+l147+l159+l169+l170+l171+l172+l173+l174+l187+l194+l197;
l255 = l58+l61+l69+l72+l73+l74+l75+l88+l89+l97+l99+l101+l102+l109+l110+l112+l134+l135+l153+l157+l158+l183+l184+l185+l186+l191+l192+l193;
l205 = -l203;
l208 = A_init.*l6.*l10.*l64.*6.0;
l209 = J_max.*T.*l6.*l10.*l64.*6.0;
l214 = -l210;
l215 = -l212;
l217 = A_init.*l64.*l156.*6.0;
l221 = J_max.*T.*l64.*l156.*6.0;
l222 = -l213;
l223 = -l216;
l224 = J_min.*l18.*l62.*l64.*1.2e+1;
l225 = -l219;
l226 = -l220;
l227 = A_max.*J_min.*J_max.*T.*l62.*l64.*1.2e+1;
l230 = l36+l37+l204;
l233 = -l64.*(l47-l49-l50+l70-l71-l119+l148+l149);
l243 = l64.*l242;
l254 = l64.*l253;
l256 = l64.*l255;
l258 = l51+l52+l54+l55+l56+l57+l80+l83+l96+l106+l107+l108+l125+l126+l127+l128+l161+l162+l163+l164+l165+l166+l167+l168+l177+l178+l179+l180+l181+l182;
l228 = -l224;
l229 = -l227;
l231 = J_min.*l230.*2.0;
l234 = l23+l24+l198+l201+l207+l208+l209+l215;
l244 = l20+l21+l29+l33+l40+l41+l42+l44+l217+l218+l221+l222+l223+l225;
l257 = -l256;
l235 = l231+l233;
l236 = J_min.*l234.*2.0;
l245 = J_min.*l244.*2.0;
l246 = l28+l34+l35+l38+l43+l48+l200+l202+l205+l206+l211+l214+l226+l228+l229;
l238 = 1.0./l235;
l247 = J_min.*l246.*2.0;
l250 = (l236-l243).^2;
l268 = l245+l257;
l239 = l238.^2;
l240 = l238.^3;
l252 = l250.^2;
l259 = l238.*(l236-l243).*(-1.0./4.0);
l265 = l64.*l238.*l258;
l270 = l238.*l268;
l272 = -l238.*(l247-l254);
l241 = l239.^2;
l260 = l239.*l250.*(3.0./8.0);
l266 = l265.*1.2e+1;
l273 = l239.*l268.*(l236-l243).*-3.0;
l275 = (l239.*l268.*(l236-l243))./4.0;
l276 = (l239.*(l236-l243).*(l247-l254))./2.0;
l278 = l240.*l250.*(l247-l254).*(3.0./4.0);
l279 = l240.*l250.*(l247-l254).*(-1.0./1.6e+1);
l262 = l241.*l252.*(3.0./2.56e+2);
l263 = l241.*l252.*(9.0./6.4e+1);
l267 = -l266;
l280 = l260+l272;
l289 = (l270-l276+(l240.*(l236-l243).^3)./8.0).^2;
l264 = -l263;
l281 = l280.^2;
l282 = l280.^3;
l290 = l289.^2;
l291 = l289.*2.7e+1;
l293 = l289./2.0;
l294 = l262+l265+l275+l279;
l283 = l281.^2;
l284 = l282.*2.0;
l286 = l282./2.7e+1;
l292 = l290.*2.7e+1;
l295 = l294.^2;
l296 = l294.^3;
l298 = l282.*l289.*4.0;
l300 = l280.*l294.*(4.0./3.0);
l301 = l280.*l294.*7.2e+1;
l306 = l280.*l289.*l294.*1.44e+2;
l285 = -l284;
l287 = -l286;
l297 = l296.*2.56e+2;
l299 = -l298;
l302 = -l300;
l303 = -l301;
l304 = l283.*l294.*1.6e+1;
l305 = l281.*l295.*1.28e+2;
l307 = -l306;
l308 = l292+l297+l299+l304+l305+l307;
l309 = sqrt(complex(l308));
l310 = l25.*l309.*3.0;
l311 = (l25.*l309)./1.8e+1;
l312 = l285+l291+l303+l310;
l314 = l287+l293+l302+l311;
l313 = sqrt(complex(l312));
l315 = l314.^(1.0./3.0);
l317 = 1.0./l314.^(1.0./6.0);
l316 = l315.^2;
l319 = l280.*l315.*6.0;
l320 = l26.*l313.*(l270-l276+(l240.*(l236-l243).^3)./8.0).*-3.0;
l321 = l26.*l313.*(l270-l276+(l240.*(l236-l243).^3)./8.0).*3.0;
l318 = l316.*9.0;
l322 = l264+l267+l273+l278+l281+l318+l319;
l323 = sqrt(complex(l322));
l324 = 1.0./l322.^(1.0./4.0);
l325 = l281.*l323;
l327 = l294.*l323.*1.2e+1;
l329 = l316.*l323.*-9.0;
l330 = (l317.*l323)./6.0;
l332 = l280.*l315.*l323.*1.2e+1;
l326 = -l325;
l331 = -l330;
l333 = l320+l326+l327+l329+l332;
l334 = l321+l326+l327+l329+l332;
l335 = sqrt(complex(l333));
l336 = sqrt(complex(l334));
l337 = (l317.*l324.*l335)./6.0;
l338 = (l317.*l324.*l336)./6.0;
t3 = [l259+l331-l338;l259+l331+l338;l259+l330-l337;l259+l330+l337];

t1 = (A_max.*(A_init+J_min.*t3)-A_init.*A_min)./(A_min.*J_max-A_max.*J_max);

l2 = t3.^2;
t6 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0-J_min.^2.*l2+A_init.^2+A_wayp.^2-A_init.*A_wayp.*2.0+A_init.*J_max.*T.*2.0+A_init.*J_max.*t1.*2.0-A_wayp.*J_max.*t1.*2.0+J_min.*J_max.*l2+J_max.^2.*T.*t1.*2.0).*(-1.0./2.0))./(J_min.*J_max.*t3);

t2 = (A_init-A_wayp+J_min.*t3-J_max.*t3-J_max.*t6+J_max.*T)./J_max;

t7 = T-t1-t2-t3-t6;

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


