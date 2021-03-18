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

function [t] = abcefg_T_P(P_init,V_init,A_init,P_wayp,~,~,~,V_min,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 15:12:31
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_max.^2;
l6 = A_max.^3;
l7 = J_min.^2;
l8 = J_min.^3;
l9 = J_max.^2;
l10 = T.^2;
l11 = -A_max;
l12 = 1.0./A_max;
l13 = 1.0./J_min;
l15 = A_init.*A_max.*J_min.*2.0;
l16 = A_min.*A_max.*J_max.*2.0;
l17 = A_min.*J_min.*J_max.*2.0;
l18 = A_max.*J_min.*J_max.*2.0;
l19 = A_init.*J_max.*V_init.*6.0;
l20 = A_max.*J_max.*V_init.*6.0;
l21 = J_min.*J_max.*V_init.*2.0;
l22 = J_min.*J_max.*V_min.*2.0;
l28 = sqrt(3.0);
l44 = A_init.*A_max.*J_max.*T.*1.8e+1;
l14 = l6.*4.0;
l23 = J_min.*l2;
l24 = J_min.*l4;
l25 = J_max.*l4;
l26 = J_min.*l5;
l27 = J_max.*l5;
l29 = l3.*1.0e+1;
l32 = -l18;
l33 = -l20;
l34 = A_init+l11;
l35 = A_min+l11;
l36 = -l21;
l37 = P_wayp.*l9.*6.0;
l38 = T.*l18;
l39 = A_init.*l5.*1.8e+1;
l40 = A_max.*l2.*2.4e+1;
l48 = -l44;
l49 = J_max.*T.*l2.*1.2e+1;
l53 = A_init.*l9.*l10.*3.0;
l54 = A_max.*l9.*l10.*3.0;
l56 = A_init.*A_min.*A_max.*l7.*l9.*2.4e+1;
l57 = A_min.*A_max.*V_init.*l7.*l9.*2.4e+1;
l61 = A_init.*l4.*l7.*l9.*1.2e+1;
l62 = A_init.*l5.*l7.*l9.*1.2e+1;
l63 = P_init.*l5.*l7.*l9.*2.4e+1;
l64 = V_init.*l5.*l7.*l9.*2.4e+1;
l30 = -l14;
l41 = -l25;
l42 = -l26;
l46 = T.*l27.*6.0;
l47 = -l40;
l50 = l34.^2;
l51 = l35.^2;
l52 = l35.^3;
l55 = -l54;
l58 = l17+l32;
l59 = -l56;
l60 = -l57;
l70 = A_init.*A_min.*A_max.*J_min.*l9.*l35.*2.4e+1;
l73 = A_init.*l9.*l26.*l35.*2.4e+1;
l74 = V_init.*l9.*l26.*l35.*2.4e+1;
l81 = l5.*l7.*l9.*l35.*1.2e+1;
l65 = l58.^2;
l66 = 1.0./l58.^3;
l69 = l5.*l9.*l52.*4.0;
l71 = -l70;
l72 = A_min.*A_max.*J_max.*l7.*l50.*1.2e+1;
l77 = A_init.*l5.*l7.*l50.*1.2e+1;
l78 = A_init.*l5.*l9.*l51.*1.2e+1;
l79 = l7.*l27.*l50.*1.2e+1;
l80 = l9.*l26.*l51.*1.2e+1;
l82 = A_init.*A_min.*J_min.*J_max.*l58.*1.2e+1;
l83 = A_init.*A_max.*J_min.*J_max.*l58.*1.2e+1;
l84 = A_max.*J_min.*J_max.*V_init.*l58.*1.2e+1;
l86 = J_max.*l26.*l35.*l50.*1.2e+1;
l88 = A_init.*A_max.*J_max.*l35.*l58.*1.2e+1;
l89 = A_max.*J_min.*J_max.*l35.*l58.*1.2e+1;
l90 = A_max.*J_min.*l50.*l58.*6.0;
l91 = A_max.*J_max.*l51.*l58.*6.0;
l92 = l22+l23+l24+l27+l36+l41+l42;
l103 = (l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).^2;
l106 = A_init.*A_min.*J_min.*J_max.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*-1.2e+1;
l109 = A_init.*A_max.*J_min.*J_max.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*1.2e+1;
l110 = A_max.*J_min.*J_max.*V_init.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*1.2e+1;
l116 = A_init.*A_max.*J_max.*l35.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*1.2e+1;
l117 = A_max.*J_min.*J_max.*l35.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*1.2e+1;
l118 = A_max.*J_min.*l50.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*6.0;
l119 = A_max.*J_max.*l51.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*6.0;
l121 = (l12.*l13.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).^3)./2.0;
l123 = A_init.*l58.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*6.0;
l125 = l35.*l58.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*6.0;
l129 = l19+l29+l30+l33+l37+l39+l46+l47+l48+l49+l53+l55;
l67 = 1.0./l65.^3;
l68 = l66.^3;
l75 = A_init.*l65.*3.0;
l76 = -l72;
l85 = -l82;
l87 = l35.*l65.*3.0;
l93 = l92.^2;
l95 = A_init.*A_min.*J_min.*J_max.*l92.*1.2e+1;
l96 = A_init.*A_max.*J_min.*J_max.*l92.*1.2e+1;
l97 = A_max.*J_min.*J_max.*V_init.*l92.*1.2e+1;
l99 = A_init.*A_max.*J_max.*l35.*l92.*1.2e+1;
l100 = A_max.*J_min.*l50.*l92.*6.0;
l101 = A_init.*l58.*l92.*6.0;
l105 = A_init.*l103.*3.0;
l111 = l35.*l103.*3.0;
l127 = l12.*l13.*l58.*l103.*(3.0./2.0);
l128 = l12.*l13.*l65.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*(3.0./2.0);
l130 = l5.*l7.*l129.*4.0;
l133 = A_init.*l92.*(l15-l16+l21-l22-l23-l24+l25+l27+l38+l42).*6.0;
l94 = A_init.*l93.*3.0;
l98 = -l95;
l131 = -l130;
l134 = l59+l61+l62+l75+l81+l83+l85+l87+l89+l128;
l135 = l134.^2;
l136 = l134.^3;
l137 = A_max.*J_min.*l66.*l134.*(2.0./3.0);
l142 = l60+l64+l71+l73+l76+l79+l80+l84+l88+l90+l91+l96+l98+l101+l106+l109+l117+l123+l125+l127;
l145 = l63+l69+l74+l77+l78+l86+l94+l97+l99+l100+l105+l110+l111+l116+l118+l119+l121+l131+l133;
l138 = -l137;
l139 = l5.*l7.*l67.*l135.*(4.0./9.0);
l140 = l6.*l8.*l68.*l136.*(8.0./2.7e+1);
l143 = A_max.*J_min.*l66.*l142.*(2.0./3.0);
l146 = A_max.*J_min.*l66.*l145;
l147 = J_min.*l11.*l66.*l145;
l148 = l5.*l7.*l67.*l134.*l142.*(2.0./3.0);
l141 = -l140;
l144 = -l143;
l149 = -l148;
l150 = l139+l144;
l153 = l140+l146+l149;
l151 = l150.^3;
l154 = l153.^2;
l152 = -l151;
l155 = l152+l154;
l156 = sqrt(complex(l155));
l157 = l141+l147+l148+l156;
l158 = l157.^(1.0./3.0);
l159 = 1.0./l158;
l161 = l158./2.0;
l162 = -l161;
l163 = l150.*l159;
l164 = l163./2.0;
l167 = l28.*(l158-l163).*-5.0e-1i;
l165 = -l164;
t6 = [l138+l158+l163;l138+l162+l165+l28.*(l158-l163).*5.0e-1i;l138+l162+l165+l167];

l2 = A_min.^2;
l3 = A_max.^2;
t7 = ((J_min.*l2+J_min.*l3-J_max.*l2-J_max.*l3+A_init.^2.*J_min-A_init.*A_max.*J_min.*2.0+A_min.*A_max.*J_max.*2.0-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_max.*J_min.*J_max.*T.*2.0-A_min.*J_min.*J_max.*t6.*2.0+A_max.*J_min.*J_max.*t6.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

l2 = A_min.^2;
l3 = A_max.^2;
t2 = (J_min.*l2-J_min.*l3-J_max.*l2+J_max.*l3+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_min.*J_min.*J_max.*t6.*2.0)./(A_max.*J_min.*J_max.*2.0);

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t3 = -(A_init-A_min+J_max.*t1)./J_min;

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


