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

function [t] = aceg_T_P(P_init,V_init,A_init,P_wayp,~,~,~,V_min,~,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 14:44:33
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = J_min.^2;
l6 = J_min.^3;
l8 = J_max.^2;
l9 = J_max.^3;
l11 = T.^2;
l12 = T.^3;
l13 = V_init.^2;
l14 = V_min.^2;
l18 = sqrt(3.0);
l19 = sqrt(6.0);
l31 = A_init.*J_min.*J_max.*V_init.*2.4e+1;
l32 = A_init.*J_min.*J_max.*V_min.*2.4e+1;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l20 = A_init.*l6.*1.2e+1;
l21 = J_min.*l3.*1.2e+1;
l22 = A_init.*l9.*1.2e+1;
l23 = J_max.*l3.*1.2e+1;
l24 = J_max.*l6.*1.2e+1;
l25 = J_min.*l9.*1.4e+1;
l26 = P_init.*l9.*2.4e+1;
l27 = P_wayp.*l9.*2.4e+1;
l29 = V_init.*l9.*1.2e+1;
l30 = V_min.*l9.*1.2e+1;
l40 = -l31;
l41 = A_init.*J_min.*l8.*3.6e+1;
l42 = A_init.*J_max.*l5.*3.6e+1;
l43 = J_min.*J_max.*l2.*3.6e+1;
l44 = A_init.*T.*l9.*2.4e+1;
l45 = J_max.*V_init.*l2.*1.2e+1;
l46 = A_init.*V_init.*l8.*2.4e+1;
l47 = J_max.*V_min.*l2.*1.2e+1;
l48 = A_init.*V_min.*l8.*2.4e+1;
l49 = J_min.*P_init.*l8.*2.4e+1;
l50 = J_min.*P_wayp.*l8.*2.4e+1;
l52 = J_min.*T.*l9.*3.6e+1;
l53 = J_max.*V_init.*l5.*1.2e+1;
l54 = J_min.*V_init.*l8.*2.4e+1;
l55 = J_max.*V_min.*l5.*1.2e+1;
l56 = J_min.*V_min.*l8.*2.4e+1;
l57 = T.*V_min.*l9.*2.4e+1;
l58 = V_init.*V_min.*l8.*2.4e+1;
l60 = J_min.*l9.*l12.*4.0;
l68 = l2.*l5.*1.8e+1;
l69 = l2.*l8.*1.8e+1;
l70 = l5.*l8.*1.9e+1;
l75 = l8.*l13.*1.2e+1;
l76 = l8.*l14.*1.2e+1;
l77 = J_min.*J_max.*T.*l2.*1.2e+1;
l78 = A_init.*J_max.*T.*l5.*2.4e+1;
l79 = A_init.*J_min.*T.*l8.*4.8e+1;
l81 = J_min.*l9.*l11.*2.4e+1;
l82 = T.*l5.*l8.*3.6e+1;
l84 = T.*l2.*l8.*1.2e+1;
l88 = A_init.*J_min.*l8.*l11.*1.2e+1;
l90 = A_init.*l9.*l11.*-1.2e+1;
l91 = l5.*l8.*l11.*1.2e+1;
l15 = l4.*3.0;
l16 = l7.*3.0;
l17 = l10.*4.0;
l28 = T.*l10.*1.2e+1;
l33 = -l22;
l34 = -l23;
l35 = -l24;
l36 = -l25;
l37 = -l26;
l39 = -l29;
l51 = T.*l24;
l61 = -l42;
l62 = -l43;
l63 = -l45;
l64 = -l48;
l65 = -l50;
l66 = -l53;
l67 = -l56;
l71 = -l57;
l72 = -l58;
l73 = l10.*l12.*-4.0;
l74 = l10.*l11.*1.2e+1;
l80 = T.*l56;
l83 = -l79;
l86 = -l81;
l87 = -l82;
l89 = -l84;
l38 = -l28;
l92 = l16+l17+l35+l36+l70;
l97 = l15+l47+l63+l72+l75+l76;
l112 = l30+l39+l44+l54+l55+l62+l66+l67+l68+l69+l74+l78+l83+l86+l91;
l113 = l21+l27+l32+l34+l37+l40+l46+l49+l60+l64+l65+l71+l73+l77+l80+l88+l89+l90;
l93 = 1.0./l92;
l98 = l20+l33+l38+l41+l51+l52+l61+l87;
l94 = l93.^2;
l95 = l93.^3;
l99 = l98.^2;
l100 = l98.^3;
l102 = l93.*l97;
l104 = (l93.*l98)./4.0;
l114 = l93.*l112;
l116 = l93.*l113;
l96 = l94.^2;
l101 = l99.^2;
l103 = l102.*1.2e+1;
l105 = -l104;
l106 = l94.*l99.*(3.0./8.0);
l107 = (l95.*l100)./8.0;
l115 = -l114;
l117 = (l94.*l98.*l112)./2.0;
l119 = l95.*l99.*l112.*(3.0./4.0);
l120 = (l95.*l99.*l112)./1.6e+1;
l121 = l94.*l98.*l113.*3.0;
l123 = (l94.*l98.*l113)./4.0;
l108 = l96.*l101.*(3.0./2.56e+2);
l109 = l96.*l101.*(9.0./6.4e+1);
l118 = -l117;
l122 = -l121;
l124 = -l123;
l125 = l106+l115;
l110 = -l108;
l111 = -l109;
l126 = l125.^2;
l127 = l125.^3;
l133 = l107+l116+l118;
l128 = l126.^2;
l129 = l127.*2.0;
l131 = l127./2.7e+1;
l134 = l133.^2;
l139 = l102+l110+l120+l124;
l130 = -l129;
l132 = -l131;
l135 = l134.^2;
l136 = l134.*2.7e+1;
l138 = l134./2.0;
l140 = l139.^2;
l141 = l139.^3;
l144 = l127.*l134.*4.0;
l146 = l125.*l139.*(4.0./3.0);
l147 = l125.*l139.*7.2e+1;
l148 = l128.*l139.*1.6e+1;
l151 = l125.*l134.*l139.*1.44e+2;
l137 = l135.*2.7e+1;
l142 = l141.*2.56e+2;
l145 = -l144;
l149 = -l148;
l150 = l126.*l140.*1.28e+2;
l143 = -l142;
l152 = l137+l143+l145+l149+l150+l151;
l153 = sqrt(complex(l152));
l154 = l18.*l153.*3.0;
l155 = (l18.*l153)./1.8e+1;
l156 = l130+l136+l147+l154;
l158 = l132+l138+l146+l155;
l157 = sqrt(complex(l156));
l159 = l158.^(1.0./3.0);
l161 = 1.0./l158.^(1.0./6.0);
l160 = l159.^2;
l163 = l125.*l159.*6.0;
l164 = l19.*l133.*l157.*3.0;
l162 = l160.*9.0;
l165 = -l164;
l166 = l103+l111+l119+l122+l126+l162+l163;
l167 = sqrt(complex(l166));
l168 = 1.0./l166.^(1.0./4.0);
l169 = l126.*l167;
l171 = l139.*l167.*1.2e+1;
l174 = l160.*l167.*-9.0;
l175 = (l161.*l167)./6.0;
l177 = l125.*l159.*l167.*1.2e+1;
l170 = -l169;
l172 = -l171;
l176 = -l175;
l178 = l164+l170+l172+l174+l177;
l179 = l165+l170+l172+l174+l177;
l180 = sqrt(complex(l178));
l181 = sqrt(complex(l179));
l182 = (l161.*l168.*l180)./6.0;
l183 = (l161.*l168.*l181)./6.0;
t3 = [l105+l176-l182;l105+l176+l182;l105+l175-l183;l105+l175+l183];

l2 = J_max.^2;
l3 = t3.^2;
t7 = -(J_max.*V_min.*2.0+l2.*l3.*2.0+J_min.^2.*l3+A_init.^2-J_max.*(V_init.*2.0+A_init.*t3.*2.0+J_min.*l3-J_min.*T.*t3.*2.0+J_max.*T.*t3.*2.0)+A_init.*J_min.*t3.*2.0-J_min.*J_max.*l3.*2.0)./(l2.*t3.*2.0-J_max.*(A_init.*2.0+J_min.*t3.*2.0)+A_init.*J_max.*2.0);

l2 = J_min.*t3;
l3 = J_max.*t7;
l4 = J_max.^2;
l5 = A_init+l2+l3;
t1 = -(J_max.*V_init.*2.0-J_max.*V_min.*2.0+l3.^2-l5.^2+l3.*(A_init+l2).*2.0+A_init.*J_max.*t3.*2.0+J_max.*l2.*t3)./(J_max.*l3.*2.0-J_max.*l5.*2.0+l4.*t3.*2.0+A_init.*J_max.*2.0);

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

