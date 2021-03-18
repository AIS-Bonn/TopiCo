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

function [t] = acdefg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 13:55:05
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l9 = J_min.^2;
l10 = J_min.^3;
l12 = J_max.^2;
l13 = J_min.^5;
l14 = J_min.*J_max.*V_init.*2.0;
l15 = J_min.*J_max.*V_wayp.*2.0;
l21 = sqrt(3.0);
l22 = sqrt(6.0);
l6 = l4.^2;
l11 = l9.^2;
l16 = J_min.*l2;
l17 = J_min.*l4;
l18 = J_max.*l4;
l19 = J_min.*l7;
l20 = J_max.*l9;
l27 = A_min.*l13.*8.0;
l29 = A_init.*J_max.*l10.*6.0;
l30 = A_min.*J_max.*l10.*3.0;
l31 = J_min.*J_max.*l5.*3.0;
l32 = A_wayp.*J_max.*l10.*6.0;
l36 = l3.*l9.*2.0;
l37 = l5.*l12.*3.0;
l39 = l8.*l9.*6.0;
l43 = A_min.*J_min.*V_init.*l12.*6.0;
l44 = A_min.*J_min.*V_wayp.*l12.*6.0;
l45 = T.*l10.*l12.*6.0;
l49 = A_wayp.*J_min.*J_max.*l5.*1.2e+1;
l54 = A_wayp.*l2.*l9.*6.0;
l55 = A_min.*l8.*l9.*4.0;
l56 = A_min.*l9.*l12.*3.0;
l57 = A_min.*l10.*l12.*4.0;
l58 = P_wayp.*l9.*l12.*1.2e+1;
l63 = A_min.*P_init.*l9.*l12.*2.4e+1;
l64 = A_min.*T.*V_init.*l9.*l12.*2.4e+1;
l25 = A_wayp.*l11.*6.0;
l33 = A_init.*A_min.*l11.*1.2e+1;
l34 = A_min.*J_max.*l11.*1.2e+1;
l35 = -l32;
l38 = l6.*l12.*4.0;
l40 = A_min.*J_max.*l16.*3.0;
l41 = A_min.*J_max.*l19.*3.0;
l42 = A_wayp.*J_max.*l17.*6.0;
l47 = l6.*l9.*8.0;
l51 = A_wayp.*V_init.*l20.*1.2e+1;
l52 = -l44;
l53 = A_wayp.*V_wayp.*l20.*1.2e+1;
l59 = T.*l2.*l20.*6.0;
l61 = -l54;
l62 = -l56;
l66 = 1.0./(l10-l20).^2;
l71 = l17.*(l10-l20).*1.2e+1;
l77 = (l14-l15-l16-l17+l18+l19).^2;
l80 = l17.*(l14-l15-l16-l17+l18+l19).*1.2e+1;
l86 = (l10-l20).*(l14-l15-l16-l17+l18+l19).*6.0;
l46 = -l34;
l48 = -l40;
l50 = T.*l34;
l60 = -l53;
l67 = l66.^2;
l68 = l66.^3;
l78 = l77.*3.0;
l84 = l25+l29+l30+l35+l45+l62;
l69 = l67.^2;
l72 = l27+l46+l57;
l87 = A_min.*l84.*2.0;
l93 = l31+l36+l37+l39+l41+l42+l43+l48+l51+l52+l58+l59+l60+l61;
l73 = l72.^2;
l74 = l72.^3;
l81 = (l66.*l72)./1.2e+1;
l90 = -l87;
l94 = A_min.*l93.*2.0;
l75 = l73.^2;
l82 = -l81;
l83 = (l67.*l73)./2.4e+1;
l85 = (l68.*l74)./2.16e+2;
l95 = -l94;
l96 = l33+l50+l71+l86+l90;
l88 = (l69.*l75)./5.76e+2;
l89 = (l69.*l75)./6.912e+3;
l97 = (l66.*l96)./3.0;
l99 = (l67.*l72.*l96)./1.8e+1;
l101 = (l68.*l73.*l96)./3.6e+1;
l102 = (l68.*l73.*l96)./4.32e+2;
l117 = l38+l47+l49+l55+l63+l64+l78+l80+l95;
l91 = -l88;
l92 = -l89;
l98 = -l97;
l100 = -l99;
l118 = l66.*l117.*4.0;
l119 = (l66.*l117)./3.0;
l103 = l83+l98;
l111 = l85+l100;
l122 = l92+l102+l119;
l104 = l103.^2;
l105 = l103.^3;
l112 = l111.^2;
l123 = l122.^2;
l124 = l122.^3;
l127 = l103.*l122.*(4.0./3.0);
l128 = l103.*l122.*7.2e+1;
l106 = l104.^2;
l107 = l105.*2.0;
l109 = l105./2.7e+1;
l113 = l112.^2;
l114 = l112.*2.7e+1;
l116 = l112./2.0;
l120 = l105.*l112.*4.0;
l125 = l124.*2.56e+2;
l131 = l104.*l123.*1.28e+2;
l132 = l103.*l112.*l122.*1.44e+2;
l108 = -l107;
l110 = -l109;
l115 = l113.*2.7e+1;
l121 = -l120;
l126 = -l125;
l129 = l106.*l122.*1.6e+1;
l130 = -l129;
l133 = l115+l121+l126+l130+l131+l132;
l134 = sqrt(complex(l133));
l135 = l21.*l134.*3.0;
l136 = (l21.*l134)./1.8e+1;
l137 = l108+l114+l128+l135;
l139 = l110+l116+l127+l136;
l138 = sqrt(complex(l137));
l140 = l139.^(1.0./3.0);
l142 = 1.0./l139.^(1.0./6.0);
l141 = l140.^2;
l144 = l103.*l140.*6.0;
l145 = l22.*l111.*l138.*3.0;
l143 = l141.*9.0;
l146 = -l145;
l147 = l91+l101+l104+l118+l143+l144;
l148 = sqrt(complex(l147));
l149 = 1.0./l147.^(1.0./4.0);
l150 = l104.*l148;
l152 = l122.*l148.*1.2e+1;
l155 = l141.*l148.*-9.0;
l156 = (l142.*l148)./6.0;
l158 = l103.*l140.*l148.*1.2e+1;
l151 = -l150;
l153 = -l152;
l157 = -l156;
l159 = l145+l151+l153+l155+l158;
l160 = l146+l151+l153+l155+l158;
l161 = sqrt(complex(l159));
l162 = sqrt(complex(l160));
l163 = (l142.*l149.*l161)./6.0;
l164 = (l142.*l149.*l162)./6.0;
t3 = [l82+l157-l163;l82+l157+l163;l82+l156-l164;l82+l156+l164];

l2 = t3.^2;
l3 = -A_wayp;
l4 = A_min+l3;
t6 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0-A_min.*l4.*2.0+J_min.^2.*l2-A_init.^2+l4.^2-J_min.*J_max.*l2+(A_min.^2.*J_max)./J_min).*(-1.0./2.0))./(A_min.*J_max);

l2 = 1.0./J_max;
t4 = l2.*(A_init+J_min.*t3-J_max.*(t3+t6-l2.*(A_min-A_wayp)+A_min./J_min)+J_max.*T);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6;l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3;l3;l3];

t1 = -(A_init+J_min.*t3)./J_max;

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


