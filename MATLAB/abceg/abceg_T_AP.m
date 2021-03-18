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

function [t] = abceg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,~,~,A_max,~,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_max.^2;
l5 = A_wayp.^2;
l6 = A_wayp.^3;
l7 = J_min.^2;
l8 = J_max.^2;
l9 = J_max.^3;
l11 = T.^2;
l12 = T.^3;
l14 = sqrt(3.0);
l25 = A_init.*A_max.*A_wayp.*J_min.*J_max.*6.0;
l10 = l8.^2;
l18 = A_init.*J_min.*l9.*3.0;
l19 = J_min.*J_max.*l3.*3.0;
l20 = A_max.*J_min.*l9.*3.0;
l22 = l3.*l8;
l23 = l6.*l8;
l28 = J_min.*P_init.*l9.*1.2e+1;
l29 = J_min.*P_wayp.*l9.*1.2e+1;
l30 = l3.*l7.*3.0;
l31 = l7.*l9.*2.0;
l32 = A_init.*A_max.*A_wayp.*l7.*6.0;
l33 = A_init.*A_max.*J_min.*l8.*6.0;
l34 = A_init.*A_max.*J_max.*l7.*6.0;
l35 = A_init.*J_min.*J_max.*l4.*6.0;
l36 = A_init.*J_min.*J_max.*l5.*3.0;
l37 = A_init.*A_wayp.*J_max.*l7.*6.0;
l38 = A_max.*J_min.*J_max.*l5.*3.0;
l39 = A_max.*A_wayp.*J_min.*l8.*6.0;
l40 = A_max.*A_wayp.*J_max.*l7.*6.0;
l41 = A_wayp.*J_min.*J_max.*l4.*6.0;
l42 = A_init.*A_max.*T.*l9.*6.0;
l43 = A_init.*J_min.*T.*l9.*6.0;
l44 = A_max.*J_min.*T.*l9.*6.0;
l45 = P_init.*l7.*l8.*6.0;
l46 = P_wayp.*l7.*l8.*6.0;
l48 = T.*l7.*l9.*6.0;
l58 = A_max.*J_min.*J_max.*l2.*9.0;
l63 = A_init.*l4.*l7.*3.0;
l64 = A_init.*l4.*l8.*3.0;
l65 = A_max.*l2.*l8.*3.0;
l66 = A_max.*l2.*l7.*6.0;
l67 = A_wayp.*l2.*l7.*3.0;
l68 = A_wayp.*l4.*l7.*3.0;
l69 = A_max.*l5.*l8.*3.0;
l70 = A_wayp.*l4.*l8.*3.0;
l71 = J_min.*l2.*l8.*3.0;
l72 = A_init.*l7.*l8.*6.0;
l73 = J_max.*l2.*l7.*6.0;
l74 = A_max.*l7.*l8.*3.0;
l75 = A_wayp.*l7.*l8.*3.0;
l76 = J_min.*l5.*l8.*3.0;
l77 = J_min.*T.*V_init.*l9.*1.2e+1;
l78 = T.*l2.*l9.*3.0;
l79 = T.*l4.*l9.*3.0;
l83 = A_init.*A_max.*J_max.*T.*l7.*1.2e+1;
l84 = A_init.*A_max.*J_min.*T.*l8.*1.8e+1;
l85 = A_max.*A_wayp.*J_max.*T.*l7.*-6.0;
l87 = J_max.*T.*l4.*l7.*3.0;
l88 = A_max.*T.*l7.*l8.*6.0;
l89 = J_min.*T.*l4.*l8.*6.0;
l91 = A_wayp.*T.*l7.*l8.*6.0;
l92 = T.*V_init.*l7.*l8.*6.0;
l104 = J_min.*T.*l2.*l8.*9.0;
l105 = J_max.*T.*l2.*l7.*9.0;
l106 = A_init.*T.*l7.*l8.*1.2e+1;
l108 = A_max.*J_min.*l9.*l11.*9.0;
l109 = J_min.*T.*l5.*l8.*-3.0;
l111 = l7.*l9.*l11.*6.0;
l115 = l7.*l9.*l12.*-2.0;
l118 = A_max.*l7.*l8.*l11.*6.0;
l120 = A_init.*l7.*l8.*l11.*-6.0;
l13 = J_min.*l10;
l15 = P_init.*l10.*6.0;
l16 = P_wayp.*l10.*6.0;
l24 = T.*V_init.*l10.*6.0;
l27 = -l20;
l49 = T.*l37;
l50 = T.*l39;
l52 = -l28;
l53 = -l30;
l54 = -l22;
l55 = -l31;
l56 = -l32;
l57 = -l33;
l59 = -l36;
l60 = -l40;
l61 = -l41;
l62 = -l44;
l80 = A_max.*l10.*l11.*3.0;
l81 = -l46;
l82 = -l48;
l86 = l11.*l18;
l93 = -l58;
l94 = -l63;
l95 = -l64;
l96 = -l69;
l97 = -l72;
l98 = -l73;
l99 = -l76;
l100 = -l77;
l101 = -l78;
l102 = -l79;
l103 = -l84;
l107 = -l87;
l112 = -l105;
l113 = -l106;
l114 = -l108;
l116 = -l111;
l119 = l11.*l75;
l17 = -l16;
l21 = T.*l13.*3.0;
l26 = l12.*l13;
l47 = l11.*l13.*3.0;
l121 = l13+l55;
l122 = 1.0./l121;
l125 = l18+l21+l27+l74+l75+l82+l97;
l131 = l34+l37+l39+l43+l47+l57+l60+l62+l71+l88+l91+l98+l99+l113+l116;
l139 = l15+l17+l19+l23+l24+l25+l26+l29+l35+l38+l42+l45+l49+l50+l52+l53+l54+l56+l59+l61+l65+l66+l67+l68+l70+l80+l81+l83+l85+l86+l89+l92+l93+l94+l95+l96+l100+l101+l102+l103+l104+l107+l109+l112+l114+l115+l118+l119+l120;
l123 = l122.^2;
l124 = l122.^3;
l126 = l125.^2;
l127 = l125.^3;
l128 = (l122.*l125)./3.0;
l132 = (l122.*l131)./3.0;
l140 = (l122.*l139)./2.0;
l129 = (l123.*l126)./9.0;
l130 = (l124.*l127)./2.7e+1;
l133 = -l132;
l134 = (l123.*l125.*l131)./6.0;
l135 = -l134;
l136 = l129+l133;
l137 = l136.^3;
l141 = l130+l135+l140;
l138 = -l137;
l142 = l141.^2;
l143 = l138+l142;
l144 = sqrt(complex(l143));
l145 = l141+l144;
l146 = l145.^(1.0./3.0);
l147 = 1.0./l146;
l148 = l146./2.0;
l149 = -l148;
l150 = l136.*l147;
l151 = -l150;
l152 = l150./2.0;
l153 = -l152;
l154 = l146+l151;
l155 = l14.*l154.*5.0e-1i;
t2 = [l128+l146+l150;l128+l149+l153-l155;l128+l149+l153+l155];

l2 = -J_max;
t3 = -(A_init-A_wayp+l2.*t2+J_max.*T)./(J_min+l2);

t7 = -(A_max-A_wayp+J_min.*t3)./J_max;

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t6 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

