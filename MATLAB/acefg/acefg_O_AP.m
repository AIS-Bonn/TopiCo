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

function [t] = acefg_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l10 = J_min.^2;
l11 = J_max.^2;
l12 = J_max.^3;
l14 = J_max.^5;
l16 = V_init.^2;
l17 = V_max.^2;
l19 = sqrt(3.0);
l20 = sqrt(6.0);
l4 = l2.^2;
l6 = l5.^2;
l9 = l7.^2;
l13 = l11.^2;
l15 = l11.^3;
l21 = J_min.*l14.*6.0;
l22 = A_init.*l14.*1.2e+1;
l23 = A_min.*l14.*8.0;
l43 = l3.*l12.*1.2e+1;
l46 = A_init.*A_min.*J_min.*l12.*3.6e+1;
l47 = A_min.*J_min.*J_max.*l8.*1.2e+1;
l48 = A_init.*J_min.*V_init.*l12.*2.4e+1;
l49 = A_min.*J_min.*V_init.*l12.*2.4e+1;
l50 = A_min.*l10.*l12.*4.0;
l51 = J_min.*l5.*l12.*6.0;
l52 = V_init.*l10.*l12.*1.2e+1;
l53 = A_init.*A_min.*J_min.*V_init.*l11.*2.4e+1;
l54 = A_min.*A_wayp.*J_max.*V_max.*l10.*2.4e+1;
l55 = J_min.*J_max.*l5.*l7.*6.0;
l57 = A_min.*l3.*l11.*8.0;
l58 = A_init.*l5.*l12.*1.2e+1;
l59 = A_min.*l2.*l12.*2.4e+1;
l60 = A_min.*l8.*l10.*8.0;
l61 = A_init.*l10.*l12.*1.2e+1;
l62 = J_min.*l3.*l11.*1.2e+1;
l63 = J_min.*l2.*l12.*3.0e+1;
l67 = A_init.*A_min.*l10.*l11.*1.2e+1;
l68 = A_init.*J_min.*l5.*l11.*1.2e+1;
l69 = A_min.*J_min.*l2.*l11.*2.4e+1;
l71 = A_min.*P_init.*l10.*l11.*2.4e+1;
l72 = A_min.*P_wayp.*l10.*l11.*2.4e+1;
l73 = J_min.*V_init.*l2.*l11.*1.2e+1;
l74 = A_init.*V_init.*l10.*l11.*2.4e+1;
l75 = J_min.*V_init.*l5.*l11.*1.2e+1;
l76 = A_min.*V_init.*l10.*l11.*2.4e+1;
l77 = J_max.*V_max.*l5.*l10.*1.2e+1;
l78 = J_min.*V_max.*l7.*l11.*1.2e+1;
l79 = J_max.*V_max.*l7.*l10.*1.2e+1;
l80 = l2.*l5.*l11.*6.0;
l81 = l5.*l7.*l10.*6.0;
l93 = l2.*l10.*l11.*1.2e+1;
l94 = l10.*l11.*l16.*1.2e+1;
l95 = l10.*l11.*l17.*1.2e+1;
l18 = l15.*3.0;
l24 = -l21;
l25 = J_min.*J_max.*l9.*6.0;
l26 = l6.*l10;
l27 = l6.*l11;
l28 = -l23;
l29 = A_init.*A_min.*l13.*2.4e+1;
l30 = A_init.*J_min.*l13.*2.4e+1;
l31 = A_min.*J_min.*l13.*1.2e+1;
l33 = J_min.*V_init.*l13.*1.2e+1;
l34 = l4.*l11.*3.0;
l35 = l5.*l13.*6.0;
l36 = l9.*l10.*3.0;
l37 = l9.*l11.*3.0;
l38 = l10.*l13.*3.0;
l44 = l2.*l13.*1.8e+1;
l56 = -l48;
l64 = -l50;
l66 = -l53;
l70 = -l55;
l82 = -l59;
l83 = -l60;
l84 = -l62;
l87 = -l68;
l88 = -l72;
l89 = -l76;
l90 = -l77;
l91 = -l79;
l92 = -l80;
l96 = -l94;
l32 = -l25;
l40 = -l30;
l42 = -l34;
l45 = -l26;
l97 = l18+l24+l38;
l115 = l43+l49+l56+l58+l69+l74+l82+l84+l87+l89;
l98 = 1.0./l97;
l102 = l22+l28+l31+l40+l61+l64;
l132 = l27+l32+l36+l37+l42+l45+l47+l54+l57+l66+l70+l71+l73+l75+l78+l81+l83+l88+l90+l91+l92+l95+l96;
l99 = l98.^2;
l100 = l98.^3;
l103 = l102.^2;
l104 = l102.^3;
l106 = (l98.*l102)./4.0;
l117 = l98.*l115;
l133 = l98.*l132;
l101 = l99.^2;
l105 = l103.^2;
l107 = -l106;
l108 = l99.*l103.*(3.0./8.0);
l109 = (l100.*l104)./8.0;
l119 = (l99.*l102.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93))./2.0;
l120 = l100.*l103.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93).*(-3.0./4.0);
l122 = (l100.*l103.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93))./1.6e+1;
l123 = l99.*l102.*l115.*3.0;
l125 = (l99.*l102.*l115)./4.0;
l134 = l133.*1.2e+1;
l111 = l101.*l105.*(3.0./2.56e+2);
l112 = l101.*l105.*(9.0./6.4e+1);
l124 = -l123;
l127 = (l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).^2;
l130 = (l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).^3.*-2.0;
l131 = (l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).^3.*(-1.0./2.7e+1);
l135 = -l134;
l136 = l109+l117+l119;
l113 = -l112;
l129 = l127.^2;
l137 = l136.^2;
l143 = l111+l122+l125+l133;
l138 = l137.^2;
l139 = l137.*2.7e+1;
l141 = l137./2.0;
l142 = l137.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).^3.*-4.0;
l144 = l143.^2;
l145 = l143.^3;
l147 = l143.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).*(-4.0./3.0);
l148 = l143.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).*-7.2e+1;
l149 = l129.*l143.*1.6e+1;
l151 = l137.*l143.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).*-1.44e+2;
l140 = l138.*2.7e+1;
l146 = l145.*2.56e+2;
l150 = l127.*l144.*1.28e+2;
l152 = l140+l142+l146+l149+l150+l151;
l153 = sqrt(complex(l152));
l154 = l19.*l153.*3.0;
l155 = (l19.*l153)./1.8e+1;
l156 = l130+l139+l148+l154;
l158 = l131+l141+l147+l155;
l157 = sqrt(complex(l156));
l159 = l158.^(1.0./3.0);
l161 = 1.0./l158.^(1.0./6.0);
l160 = l159.^2;
l164 = l159.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).*6.0;
l165 = l20.*l136.*l157.*3.0;
l162 = l160.*9.0;
l166 = -l165;
l167 = l113+l120+l124+l127+l135+l162+l164;
l168 = sqrt(complex(l167));
l169 = 1.0./l167.^(1.0./4.0);
l170 = l127.*l168;
l172 = l143.*l168.*1.2e+1;
l174 = l160.*l168.*-9.0;
l175 = (l161.*l168)./6.0;
l178 = l159.*l168.*(l108+l98.*(l29+l33-l35-l44-l46+l51-l52+l63+l67-l93)).*1.2e+1;
l171 = -l170;
l176 = -l175;
l179 = l165+l171+l172+l174+l178;
l180 = l166+l171+l172+l174+l178;
l181 = sqrt(complex(l179));
l182 = sqrt(complex(l180));
l183 = (l161.*l169.*l181)./6.0;
l184 = (l161.*l169.*l182)./6.0;
t1 = [l107+l176-l183;l107+l176+l183;l107+l175-l184;l107+l175+l184];

l2 = J_max.*t1;
l3 = -A_min;
l4 = -A_wayp;
l5 = 1.0./J_max;
l6 = A_min+l4;
l7 = A_init+l2+l3;
t6 = ((J_min.*V_init.*2.0-J_min.*V_max.*2.0+A_wayp.*l4+l7.^2-l7.*(A_init+l2).*2.0+A_init.*J_min.*t1.*2.0+J_min.*l2.*t1+J_min.*l5.*l6.^2-A_min.*J_min.*l5.*l6.*2.0).*(-1.0./2.0))./(A_min.*J_min);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6;l6;l6];

t3 = -(A_init-A_min+J_max.*t1)./J_min;

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


