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

function [t] = acdefg_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,V_min,~,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 16:16:41
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l7 = J_min.^2;
l8 = J_min.^3;
l10 = J_max.^2;
l11 = J_min.^5;
l12 = J_max.^3;
l15 = J_min.^7;
l16 = J_max.^5;
l18 = J_max.*V_init.*2.0;
l19 = J_max.*V_max.*2.0;
l20 = 1.0./A_min;
l22 = -J_min;
l23 = -J_max;
l25 = J_min.*J_max.*V_min.*2.0;
l29 = sqrt(3.0);
l30 = J_min.*J_max.*V_max.*-2.0;
l6 = l4.^2;
l9 = l7.^2;
l13 = l7.^3;
l14 = l10.^2;
l17 = l10.^3;
l21 = 1.0./l4;
l24 = -l18;
l26 = J_min.*l19;
l27 = J_min.*l4;
l28 = J_max.*l4;
l31 = J_min+l23;
l32 = J_max+l22;
l33 = l4.*l23;
l34 = sqrt(complex(l22));
l35 = l34.^3;
l36 = l34.^5;
l37 = l34.^7;
l38 = l31.^2;
l40 = l31.^5;
l42 = sqrt(complex(l32));
l47 = l2+l19+l24;
l52 = l25+l27+l28;
l129 = l25+l27+l30+l33;
l39 = l38.^2;
l41 = l35.^3;
l43 = l42.^3;
l44 = l42.^7;
l46 = A_init.*l42;
l50 = l26.*l42;
l51 = A_min.*J_min.*J_max.*T.*l42.*2.0;
l53 = sqrt(complex(l47));
l54 = A_min.*l9.*l17.*l38.*4.0;
l55 = A_min.*l12.*l15.*l38.*4.0;
l56 = l22.*l47;
l57 = A_init.*l5.*l15.*l38.*1.2e+1;
l58 = A_min.*l3.*l15.*l38.*1.6e+1;
l60 = A_min.*l11.*l16.*l38.*1.2e+1;
l61 = A_min.*l13.*l14.*l38.*1.2e+1;
l62 = A_init.*A_min.*J_max.*V_min.*l15.*l38.*2.4e+1;
l64 = A_init.*J_max.*l5.*l13.*l38.*6.0e+1;
l65 = A_min.*J_max.*l3.*l13.*l38.*6.0e+1;
l66 = A_min.*P_init.*l9.*l16.*l38.*2.4e+1;
l67 = A_min.*P_init.*l10.*l15.*l38.*2.4e+1;
l68 = A_min.*P_wayp.*l9.*l10.*l40.*2.4e+1;
l69 = l4.*l9.*l16.*l38.*1.2e+1;
l70 = l4.*l10.*l15.*l38.*1.2e+1;
l71 = l5.*l8.*l16.*l38.*1.2e+1;
l72 = l5.*l10.*l13.*l38.*1.2e+1;
l73 = l6.*l7.*l16.*l38.*1.6e+1;
l74 = l6.*l10.*l11.*l38.*1.6e+1;
l75 = l4.*l11.*l14.*l38.*3.6e+1;
l76 = l4.*l12.*l13.*l38.*3.6e+1;
l77 = l5.*l9.*l14.*l38.*3.6e+1;
l78 = l5.*l11.*l12.*l38.*3.6e+1;
l79 = l6.*l8.*l14.*l38.*4.8e+1;
l80 = l6.*l9.*l12.*l38.*4.8e+1;
l81 = A_init.*A_min.*V_init.*l8.*l16.*l38.*2.4e+1;
l82 = A_init.*A_min.*V_init.*l9.*l14.*l38.*2.4e+1;
l83 = A_init.*A_min.*V_init.*l11.*l12.*l38.*2.4e+1;
l84 = A_init.*A_min.*V_init.*l10.*l13.*l38.*2.4e+1;
l85 = A_init.*A_min.*V_min.*l8.*l16.*l38.*2.4e+1;
l86 = A_min.*l3.*l9.*l12.*l38.*4.0;
l90 = A_min.*P_init.*l11.*l14.*l38.*7.2e+1;
l91 = A_min.*P_init.*l12.*l13.*l38.*7.2e+1;
l102 = A_init.*A_min.*V_min.*l11.*l12.*l38.*9.6e+1;
l103 = A_init.*A_min.*V_min.*l10.*l13.*l38.*9.6e+1;
l105 = A_init.*l5.*l7.*l16.*l38.*1.2e+1;
l106 = A_init.*l5.*l8.*l14.*l38.*1.2e+1;
l107 = A_min.*l3.*l8.*l14.*l38.*1.2e+1;
l108 = l2.*l13.*l28.*l38.*2.4e+1;
l109 = A_init.*l5.*l9.*l12.*l38.*4.8e+1;
l110 = A_min.*l3.*l10.*l11.*l38.*6.0e+1;
l111 = V_init.*l4.*l10.*l13.*l38.*2.4e+1;
l112 = V_init.*l4.*l8.*l16.*l38.*4.8e+1;
l113 = V_min.*l4.*l8.*l16.*l38.*2.4e+1;
l114 = V_min.*l4.*l11.*l12.*l38.*2.4e+1;
l115 = V_min.*l4.*l9.*l14.*l38.*4.8e+1;
l121 = A_init.*l5.*l10.*l11.*l38.*9.6e+1;
l123 = V_init.*l4.*l11.*l12.*l38.*9.6e+1;
l124 = V_init.*l4.*l9.*l14.*l38.*1.2e+2;
l128 = l2.*l4.*l8.*l14.*l38.*3.6e+1;
l131 = l2.*l4.*l10.*l11.*l38.*8.4e+1;
l132 = l2.*l4.*l9.*l12.*l38.*9.6e+1;
l135 = l129.^2;
l136 = l42.*l52;
l147 = A_init.*A_min.*l13.*l38.*l129.*1.2e+1;
l148 = A_min.*J_max.*l13.*l38.*l129.*1.2e+1;
l149 = J_max.*V_init.*l13.*l38.*l129.*1.2e+1;
l150 = A_init.*A_min.*J_max.*l11.*l38.*l129.*4.8e+1;
l156 = l2.*l13.*l38.*l129.*1.2e+1;
l158 = J_max.*l2.*l11.*l38.*l129.*4.2e+1;
l159 = A_min.*l8.*l14.*l38.*l129.*1.2e+1;
l160 = l11.*l28.*l38.*l129.*1.2e+1;
l161 = A_min.*l9.*l12.*l38.*l129.*3.6e+1;
l162 = A_min.*l10.*l11.*l38.*l129.*3.6e+1;
l163 = V_init.*l8.*l14.*l38.*l129.*2.4e+1;
l164 = V_init.*l10.*l11.*l38.*l129.*4.8e+1;
l165 = V_init.*l9.*l12.*l38.*l129.*6.0e+1;
l166 = V_min.*l8.*l14.*l38.*l129.*1.2e+1;
l167 = V_min.*l10.*l11.*l38.*l129.*1.2e+1;
l168 = V_min.*l9.*l12.*l38.*l129.*2.4e+1;
l178 = A_init.*A_min.*l7.*l14.*l38.*l129.*1.2e+1;
l179 = A_init.*A_min.*l9.*l10.*l38.*l129.*4.8e+1;
l184 = l2.*l8.*l12.*l38.*l129.*1.8e+1;
l185 = l2.*l9.*l10.*l38.*l129.*4.8e+1;
l186 = l4.*l7.*l14.*l38.*l129.*2.4e+1;
l187 = l4.*l9.*l10.*l38.*l129.*4.8e+1;
l188 = l4.*l8.*l12.*l38.*l129.*6.0e+1;
l45 = l43.^3;
l49 = A_min.*J_min.*l46.*2.0;
l59 = -l55;
l63 = -l60;
l87 = -l64;
l88 = -l65;
l89 = -l67;
l92 = -l70;
l93 = -l72;
l94 = -l74;
l95 = -l75;
l96 = -l77;
l97 = -l79;
l98 = l56.^(3.0./2.0);
l99 = -l82;
l100 = -l83;
l101 = -l85;
l104 = -l86;
l116 = -l90;
l117 = -l103;
l118 = -l106;
l119 = -l107;
l120 = -l109;
l122 = -l111;
l125 = -l113;
l126 = -l114;
l127 = -l124;
l130 = -l128;
l133 = -l131;
l134 = l34.*l53;
l137 = A_min.*l35.*l53.*2.0;
l139 = -l136;
l143 = l11.*l38.*l135.*3.0;
l153 = -l147;
l154 = -l148;
l155 = -l149;
l157 = J_max.*l9.*l38.*l135.*1.5e+1;
l169 = -l158;
l170 = -l160;
l171 = -l161;
l172 = -l165;
l173 = -l166;
l174 = -l167;
l175 = l7.*l12.*l38.*l135.*9.0;
l176 = l8.*l10.*l38.*l135.*2.1e+1;
l183 = -l179;
l192 = -l184;
l193 = -l188;
l138 = -l134;
l140 = A_min.*J_max.*l134.*2.0;
l141 = A_min.*l9.*l44.*l98.*4.0;
l142 = A_min.*J_max.*l8.*l44.*l98.*1.2e+1;
l145 = -l143;
l146 = A_min.*l7.*l10.*l44.*l98.*4.0;
l177 = A_min.*J_max.*V_init.*l41.*l45.*l53.*2.4e+1;
l181 = -l176;
l182 = A_min.*l2.*l41.*l45.*l53.*1.2e+1;
l189 = A_min.*J_max.*l2.*l37.*l45.*l53.*1.2e+1;
l190 = A_init.*l28.*l37.*l45.*l53.*2.4e+1;
l191 = A_min.*V_init.*l10.*l37.*l45.*l53.*2.4e+1;
l195 = A_init.*l4.*l10.*l36.*l45.*l53.*2.4e+1;
l199 = A_init.*l37.*l45.*l53.*l129.*1.2e+1;
l201 = A_init.*J_max.*l36.*l45.*l53.*l129.*1.2e+1;
l203 = l54+l59+l61+l63;
l144 = -l142;
l151 = l46+l138;
l152 = -l146;
l194 = -l190;
l196 = -l195;
l200 = -l199;
l202 = -l201;
l205 = 1.0./l203;
l213 = l49+l50+l51+l137+l139+l140;
l180 = l151.^2;
l204 = A_min.*l8.*l10.*l44.*l47.*l151.*1.2e+1;
l206 = l205.^2;
l207 = l205.^3;
l209 = A_init.*A_min.*J_max.*l37.*l39.*l53.*l151.*2.4e+1;
l211 = l4.*l10.*l36.*l39.*l53.*l151.*2.4e+1;
l212 = J_max.*l36.*l39.*l53.*l129.*l151.*1.2e+1;
l214 = l213.^2;
l215 = l213.^3;
l222 = A_min.*J_max.*l13.*l43.*l213.*1.2e+1;
l223 = J_max.*V_init.*l13.*l43.*l213.*1.2e+1;
l224 = l8.*l16.*l43.*l213.*6.0;
l225 = l10.*l13.*l43.*l213.*6.0;
l229 = l2.*l13.*l43.*l213.*1.2e+1;
l231 = l9.*l14.*l43.*l213.*1.8e+1;
l232 = l11.*l12.*l43.*l213.*1.8e+1;
l235 = J_max.*l2.*l11.*l43.*l213.*4.2e+1;
l236 = A_min.*l8.*l14.*l43.*l213.*1.2e+1;
l237 = l11.*l28.*l43.*l213.*1.2e+1;
l238 = A_min.*l9.*l12.*l43.*l213.*3.6e+1;
l239 = A_min.*l10.*l11.*l43.*l213.*3.6e+1;
l240 = V_init.*l8.*l14.*l43.*l213.*2.4e+1;
l241 = V_init.*l10.*l11.*l43.*l213.*4.8e+1;
l242 = V_init.*l9.*l12.*l43.*l213.*6.0e+1;
l243 = V_min.*l8.*l14.*l43.*l213.*1.2e+1;
l244 = V_min.*l10.*l11.*l43.*l213.*1.2e+1;
l245 = V_min.*l9.*l12.*l43.*l213.*2.4e+1;
l255 = l2.*l8.*l12.*l43.*l213.*1.8e+1;
l256 = l2.*l9.*l10.*l43.*l213.*4.8e+1;
l258 = l4.*l7.*l14.*l43.*l213.*1.2e+1;
l259 = l4.*l8.*l12.*l43.*l213.*3.6e+1;
l260 = l4.*l9.*l10.*l43.*l213.*3.6e+1;
l269 = A_init.*l37.*l39.*l53.*l213.*1.2e+1;
l270 = A_init.*J_max.*l36.*l39.*l53.*l213.*1.2e+1;
l273 = l11.*l43.*l129.*l213.*6.0;
l275 = J_max.*l9.*l43.*l129.*l213.*2.4e+1;
l276 = l7.*l12.*l43.*l129.*l213.*1.2e+1;
l277 = l8.*l10.*l43.*l129.*l213.*3.0e+1;
l280 = J_max.*l36.*l44.*l53.*l151.*l213.*1.2e+1;
l197 = A_init.*A_min.*l9.*l39.*l180.*1.2e+1;
l198 = l8.*l28.*l39.*l180.*1.2e+1;
l208 = l8.*l39.*l129.*l180.*6.0;
l210 = -l209;
l216 = l11.*l31.*l214.*3.0;
l217 = J_max.*l9.*l31.*l214.*9.0;
l219 = l7.*l12.*l31.*l214.*3.0;
l221 = l8.*l10.*l31.*l214.*9.0;
l227 = -l222;
l228 = -l223;
l230 = -l225;
l233 = l7.*l14.*l20.*l31.*l214.*3.0;
l234 = -l231;
l247 = l8.*l12.*l20.*l31.*l214.*9.0;
l248 = l9.*l10.*l20.*l31.*l214.*9.0;
l249 = -l235;
l250 = -l237;
l251 = -l238;
l252 = -l242;
l253 = -l243;
l254 = -l244;
l261 = -l255;
l262 = -l259;
l263 = (l9.*l21.*l42.*l215)./2.0;
l264 = (J_min.*l12.*l21.*l42.*l215)./2.0;
l265 = J_max.*l8.*l21.*l42.*l215.*(3.0./2.0);
l267 = l7.*l10.*l21.*l42.*l215.*(3.0./2.0);
l271 = -l269;
l272 = -l270;
l274 = -l273;
l278 = -l277;
l279 = l8.*l44.*l180.*l213.*6.0;
l218 = -l217;
l220 = -l219;
l226 = J_max.*l20.*l216;
l246 = -l233;
l257 = -l248;
l266 = -l263;
l268 = -l267;
l281 = l69+l76+l92+l95+l224+l230+l232+l234;
l282 = l281.^2;
l283 = l281.^3;
l284 = (l205.*l281)./3.0;
l287 = l71+l78+l93+l96+l154+l159+l162+l171+l226+l227+l236+l239+l246+l247+l251+l257;
l295 = l57+l58+l62+l66+l68+l73+l80+l81+l84+l87+l88+l89+l91+l94+l97+l99+l100+l101+l102+l104+l105+l108+l110+l112+l115+l116+l117+l118+l119+l120+l121+l122+l123+l125+l126+l127+l130+l132+l133+l141+l144+l145+l150+l152+l153+l155+l156+l157+l163+l164+l168+l169+l170+l172+l173+l174+l175+l177+l178+l181+l182+l183+l185+l186+l187+l189+l191+l192+l193+l194+l196+l197+l198+l200+l202+l204+l208+l210+l211+l212+l216+l218+l220+l221+l228+l229+l240+l241+l245+l249+l250+l252+l253+l254+l256+l258+l260+l261+l262+l264+l265+l266+l268+l271+l272+l274+l275+l276+l278+l279+l280;
l285 = (l206.*l282)./9.0;
l286 = (l207.*l283)./2.7e+1;
l288 = (l205.*l287)./3.0;
l290 = (l206.*l281.*l287)./6.0;
l296 = (l205.*l295)./2.0;
l289 = -l288;
l291 = -l290;
l292 = l285+l289;
l297 = l286+l291+l296;
l293 = l292.^3;
l298 = l297.^2;
l294 = -l293;
l299 = l294+l298;
l300 = sqrt(complex(l299));
l301 = l297+l300;
l302 = l301.^(1.0./3.0);
l303 = 1.0./l302;
l304 = l302./2.0;
l305 = -l304;
l306 = l292.*l303;
l307 = -l306;
l308 = l306./2.0;
l309 = -l308;
l310 = l302+l307;
l311 = l29.*l310.*5.0e-1i;
t4 = [l284+l302+l306;l284+l305+l309-l311;l284+l305+l309+l311];

l2 = A_init.^2;
l3 = A_min.^2;
l4 = J_max.*V_init.*2.0;
l5 = J_max.*V_max.*2.0;
l6 = -J_min;
l7 = -l4;
l8 = J_max+l6;
l9 = sqrt(complex(l8));
l10 = l2+l5+l7;
l11 = sqrt(complex(l10));
t7 = (-l9.*(J_min.*l3+J_max.*l3+J_min.*J_max.*V_min.*2.0+A_min.*J_min.*J_max.*t4.*2.0)+J_min.*l5.*l9+A_min.*l6.^(3.0./2.0).*l11.*2.0+A_min.*J_max.*sqrt(complex(l6)).*l11.*2.0+A_init.*A_min.*J_min.*l9.*2.0+A_min.*J_min.*J_max.*T.*l9.*2.0)./(A_min.*J_min.*J_max.*l9.*2.0);

l2 = A_init.^2;
l3 = A_min.^2;
l4 = J_min.^2;
l5 = J_max.^2;
l6 = J_min.*J_max.*V_max.*2.0;
l7 = J_min.*l2;
l8 = J_max.*l3.*2.0;
l9 = V_min.*l5.*2.0;
l10 = A_min.*J_min.*l5.*2.0;
l11 = A_min.*J_max.*l4.*2.0;
l12 = J_max.*V_min.*l4.*2.0;
l13 = J_min.*V_max.*l5.*2.0;
l14 = l2.*l4;
l15 = l3.*l4;
l16 = l3.*l5;
l17 = -l11;
l20 = l6+l7+l8+l9;
l18 = l10+l17;
l21 = J_min.*l20;
l19 = 1.0./l18;
l22 = -l21;
l23 = l12+l13+l14+l15+l16+l22;
l24 = l19.*l23;
l25 = -l24;
t6 = [l25;l25;l25];

l2 = A_min.^2;
t3 = sqrt(complex((J_min-J_max).*(J_min.*l2-J_max.*l2+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_min.*J_min.*J_max.*t6.*2.0)))./(J_min.^2-J_min.*J_max);

t1 = -(A_init+J_min.*t3)./J_max;

t5 = -(A_init-A_min+J_min.*t3+J_max.*t1)./J_min;

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

