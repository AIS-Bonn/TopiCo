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

function [t] = acdefg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
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
l14 = J_min.*J_max.*V_init.*2.0;
l15 = J_min.*J_max.*V_wayp.*2.0;
l21 = sqrt(3.0);
l22 = sqrt(6.0);
l6 = l4.^2;
l12 = l10.^2;
l16 = J_max.*l2;
l17 = J_min.*l4;
l18 = J_max.*l4;
l19 = J_min.*l7;
l20 = J_min.*l10;
l26 = A_min.*l13.*8.0;
l28 = A_init.*J_min.*l11.*3.0;
l29 = A_min.*J_min.*l11.*3.0;
l30 = l3.*l10;
l34 = l5.*l9.*3.0;
l37 = A_min.*J_max.*V_init.*l9.*6.0;
l38 = A_min.*J_max.*V_wayp.*l9.*6.0;
l39 = P_wayp.*l9.*l10.*6.0;
l40 = T.*l9.*l11.*3.0;
l43 = A_wayp.*J_min.*J_max.*l5.*1.2e+1;
l46 = A_min.*l7.*l9.*3.0;
l47 = A_wayp.*l4.*l9.*3.0;
l48 = A_min.*l8.*l9.*4.0;
l49 = A_min.*l9.*l10.*3.0;
l50 = A_min.*l9.*l11.*4.0;
l53 = A_min.*P_init.*l9.*l10.*2.4e+1;
l54 = A_min.*T.*V_init.*l9.*l10.*2.4e+1;
l31 = A_init.*A_min.*l12.*1.2e+1;
l32 = -l29;
l33 = A_min.*J_min.*l12.*1.2e+1;
l35 = l6.*l10.*4.0;
l36 = A_min.*J_min.*l16.*3.0;
l42 = l6.*l9.*8.0;
l45 = -l37;
l51 = T.*l2.*l20.*3.0;
l52 = -l46;
l56 = 1.0./(l11-l20).^2;
l60 = l18.*(l11-l20).*-6.0;
l61 = A_min.*A_wayp.*J_min.*(l11-l20).*-1.2e+1;
l70 = (l14-l15-l16-l17+l18+l19).^2;
l72 = A_min.*A_wayp.*J_min.*(l14-l15-l16-l17+l18+l19).*-1.2e+1;
l73 = l18.*(l14-l15-l16-l17+l18+l19).*-6.0;
l78 = (l11-l20).*(l14-l15-l16-l17+l18+l19).*6.0;
l41 = -l33;
l44 = T.*l33;
l57 = l56.^2;
l58 = l56.^3;
l66 = l28+l32+l40+l49;
l71 = l70.*3.0;
l83 = l30+l34+l36+l38+l39+l45+l47+l51+l52;
l59 = l57.^2;
l62 = l26+l41+l50;
l68 = A_min.*l66.*4.0;
l84 = A_min.*l83.*4.0;
l63 = l62.^2;
l64 = l62.^3;
l69 = -l68;
l74 = (l56.*l62)./1.2e+1;
l85 = -l84;
l65 = l63.^2;
l75 = -l74;
l76 = (l57.*l63)./2.4e+1;
l77 = (l58.*l64)./2.16e+2;
l86 = l31+l44+l60+l61+l69+l78;
l107 = l35+l42+l43+l48+l53+l54+l71+l72+l73+l85;
l79 = (l59.*l65)./5.76e+2;
l80 = (l59.*l65)./6.912e+3;
l87 = (l56.*l86)./3.0;
l89 = (l57.*l62.*l86)./1.8e+1;
l91 = (l58.*l63.*l86)./3.6e+1;
l92 = (l58.*l63.*l86)./4.32e+2;
l108 = l56.*l107.*4.0;
l109 = (l56.*l107)./3.0;
l81 = -l79;
l82 = -l80;
l88 = -l87;
l90 = -l89;
l93 = l76+l88;
l101 = l77+l90;
l112 = l82+l92+l109;
l94 = l93.^2;
l95 = l93.^3;
l102 = l101.^2;
l113 = l112.^2;
l114 = l112.^3;
l117 = l93.*l112.*(4.0./3.0);
l118 = l93.*l112.*7.2e+1;
l96 = l94.^2;
l97 = l95.*2.0;
l99 = l95./2.7e+1;
l103 = l102.^2;
l104 = l102.*2.7e+1;
l106 = l102./2.0;
l110 = l95.*l102.*4.0;
l115 = l114.*2.56e+2;
l121 = l94.*l113.*1.28e+2;
l122 = l93.*l102.*l112.*1.44e+2;
l98 = -l97;
l100 = -l99;
l105 = l103.*2.7e+1;
l111 = -l110;
l116 = -l115;
l119 = l96.*l112.*1.6e+1;
l120 = -l119;
l123 = l105+l111+l116+l120+l121+l122;
l124 = sqrt(complex(l123));
l125 = l21.*l124.*3.0;
l126 = (l21.*l124)./1.8e+1;
l127 = l98+l104+l118+l125;
l129 = l100+l106+l117+l126;
l128 = sqrt(complex(l127));
l130 = l129.^(1.0./3.0);
l132 = 1.0./l129.^(1.0./6.0);
l131 = l130.^2;
l134 = l93.*l130.*6.0;
l135 = l22.*l101.*l128.*3.0;
l133 = l131.*9.0;
l136 = -l135;
l137 = l81+l91+l94+l108+l133+l134;
l138 = sqrt(complex(l137));
l139 = 1.0./l137.^(1.0./4.0);
l140 = l94.*l138;
l142 = l112.*l138.*1.2e+1;
l145 = l131.*l138.*-9.0;
l146 = (l132.*l138)./6.0;
l148 = l93.*l130.*l138.*1.2e+1;
l141 = -l140;
l143 = -l142;
l147 = -l146;
l149 = l135+l141+l143+l145+l148;
l150 = l136+l141+l143+l145+l148;
l151 = sqrt(complex(l149));
l152 = sqrt(complex(l150));
l153 = (l132.*l139.*l151)./6.0;
l154 = (l132.*l139.*l152)./6.0;
t3 = [l75+l147-l153;l75+l147+l153;l75+l146-l154;l75+l146+l154];

l2 = A_min.^2;
l3 = t3.^2;
t6 = (J_min.*l2-J_max.*l2+A_init.^2.*J_max-A_wayp.^2.*J_min-J_max.^3.*l3-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_wayp.*2.0+J_min.*J_max.^2.*l3)./(A_min.*J_min.*J_max.*2.0);

l2 = 1.0./J_min;
t4 = l2.*(A_init+J_max.*t3-J_min.*(t3+t6+A_min.*l2-(A_min-A_wayp)./J_max)+J_min.*T);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6;l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3;l3;l3];

t1 = -(A_init+J_max.*t3)./J_min;

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


