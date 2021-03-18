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

function [t] = abceg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,A_max,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l7 = A_wayp.^2;
l8 = A_wayp.^3;
l10 = J_min.^2;
l11 = J_max.^2;
l12 = J_max.^3;
l14 = J_max.^5;
l16 = V_init.^2;
l17 = V_wayp.^2;
l19 = sqrt(3.0);
l20 = sqrt(6.0);
l4 = l2.^2;
l6 = l5.^2;
l9 = l7.^2;
l13 = l11.^2;
l15 = l11.^3;
l21 = J_min.*l14.*6.0;
l22 = A_max.*l14.*8.0;
l23 = A_wayp.*l14.*1.2e+1;
l41 = l8.*l12.*1.2e+1;
l43 = A_max.*A_wayp.*J_min.*l12.*3.6e+1;
l44 = A_max.*J_min.*V_wayp.*l12.*2.4e+1;
l45 = A_wayp.*J_min.*V_wayp.*l12.*2.4e+1;
l46 = A_max.*l10.*l12.*4.0;
l47 = J_min.*l5.*l12.*6.0;
l48 = V_wayp.*l10.*l12.*1.2e+1;
l49 = A_init.*A_max.*J_max.*V_init.*l10.*2.4e+1;
l50 = A_max.*A_wayp.*J_min.*V_wayp.*l11.*2.4e+1;
l52 = A_max.*l3.*l10.*8.0;
l53 = A_max.*l8.*l11.*8.0;
l54 = A_wayp.*l5.*l12.*1.2e+1;
l55 = A_max.*l7.*l12.*2.4e+1;
l57 = A_wayp.*l10.*l12.*1.2e+1;
l58 = J_min.*l8.*l11.*1.2e+1;
l59 = J_min.*l7.*l12.*3.0e+1;
l61 = A_max.*A_wayp.*l10.*l11.*1.2e+1;
l62 = A_wayp.*J_min.*l5.*l11.*1.2e+1;
l63 = A_max.*J_min.*l7.*l11.*2.4e+1;
l64 = A_max.*P_init.*l10.*l11.*2.4e+1;
l65 = A_max.*P_wayp.*l10.*l11.*2.4e+1;
l66 = J_max.*V_init.*l2.*l10.*1.2e+1;
l67 = J_max.*V_init.*l5.*l10.*1.2e+1;
l68 = J_min.*V_wayp.*l5.*l11.*1.2e+1;
l69 = A_max.*V_wayp.*l10.*l11.*2.4e+1;
l70 = J_min.*V_wayp.*l7.*l11.*1.2e+1;
l71 = A_wayp.*V_wayp.*l10.*l11.*2.4e+1;
l72 = l2.*l5.*l10.*6.0;
l73 = l5.*l7.*l11.*6.0;
l86 = l7.*l10.*l11.*1.2e+1;
l87 = l10.*l11.*l16.*1.2e+1;
l88 = l10.*l11.*l17.*1.2e+1;
l18 = l15.*3.0;
l24 = -l21;
l25 = l6.*l10;
l26 = l6.*l11;
l27 = -l23;
l28 = A_max.*A_wayp.*l13.*2.4e+1;
l29 = A_max.*J_min.*l13.*1.2e+1;
l30 = A_wayp.*J_min.*l13.*2.4e+1;
l31 = J_min.*V_wayp.*l13.*1.2e+1;
l32 = l4.*l10.*3.0;
l33 = l5.*l13.*6.0;
l34 = l9.*l11.*3.0;
l35 = l10.*l13.*3.0;
l42 = l7.*l13.*1.8e+1;
l51 = -l45;
l75 = -l55;
l76 = -l57;
l77 = -l58;
l80 = -l62;
l84 = -l69;
l37 = -l29;
l90 = l18+l24+l35;
l109 = l41+l44+l51+l54+l63+l71+l75+l77+l80+l84;
l91 = 1.0./l90;
l95 = l22+l27+l30+l37+l46+l76;
l92 = l91.^2;
l93 = l91.^3;
l96 = l95.^2;
l97 = l95.^3;
l99 = (l91.*l95)./4.0;
l111 = l91.*l109;
l126 = -l91.*(l25-l26-l32+l34-l49+l50+l52-l53+l64-l65+l66+l67-l68-l70-l72+l73-l87+l88);
l128 = l91.*(l25-l26-l32+l34-l49+l50+l52-l53+l64-l65+l66+l67-l68-l70-l72+l73-l87+l88).*1.2e+1;
l94 = l92.^2;
l98 = l96.^2;
l100 = -l99;
l101 = l92.*l96.*(3.0./8.0);
l102 = (l93.*l97)./8.0;
l113 = l93.*l96.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86).*(-3.0./4.0);
l115 = (l93.*l96.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86))./1.6e+1;
l116 = l92.*l95.*l109.*3.0;
l117 = (l92.*l95.*l109)./4.0;
l105 = l94.*l98.*(3.0./2.56e+2);
l106 = l94.*l98.*(9.0./6.4e+1);
l118 = -l117;
l121 = (l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).^2;
l124 = (l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).^3.*-2.0;
l125 = (l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).^3.*(-1.0./2.7e+1);
l130 = (l102-l111+(l92.*l95.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86))./2.0).^2;
l107 = -l106;
l123 = l121.^2;
l131 = l130.^2;
l132 = l130.*2.7e+1;
l134 = l130./2.0;
l135 = l130.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).^3.*-4.0;
l136 = l105+l115+l118+l126;
l133 = l131.*2.7e+1;
l137 = l136.^2;
l138 = l136.^3;
l140 = l136.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).*(-4.0./3.0);
l141 = l136.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).*-7.2e+1;
l142 = l123.*l136.*1.6e+1;
l144 = l130.*l136.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).*-1.44e+2;
l139 = l138.*2.56e+2;
l143 = l121.*l137.*1.28e+2;
l145 = l133+l135+l139+l142+l143+l144;
l146 = sqrt(complex(l145));
l147 = l19.*l146.*3.0;
l148 = (l19.*l146)./1.8e+1;
l149 = l124+l132+l141+l147;
l151 = l125+l134+l140+l148;
l150 = sqrt(complex(l149));
l152 = l151.^(1.0./3.0);
l154 = 1.0./l151.^(1.0./6.0);
l153 = l152.^2;
l157 = l152.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).*6.0;
l158 = l20.*l150.*(l102-l111+(l92.*l95.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86))./2.0).*-3.0;
l159 = l20.*l150.*(l102-l111+(l92.*l95.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86))./2.0).*3.0;
l155 = l153.*9.0;
l160 = l107+l113+l116+l121+l128+l155+l157;
l161 = real(sqrt(complex(l160)));
l162 = 1.0./l160.^(1.0./4.0);
l163 = l121.*l161;
l165 = l136.*l161.*1.2e+1;
l167 = l153.*l161.*-9.0;
l168 = (l154.*l161)./6.0;
l171 = l152.*l161.*(l101+l91.*(l28+l31-l33-l42-l43+l47-l48+l59+l61-l86)).*1.2e+1;
l164 = -l163;
l169 = -l168;
l172 = l158+l164+l165+l167+l171;
l173 = l159+l164+l165+l167+l171;
l174 = sqrt(complex(l172));
l175 = sqrt(complex(l173));
l176 = (l154.*l162.*l174)./6.0;
l177 = (l154.*l162.*l175)./6.0;
t7 = [l100+l169-l177;l100+l169+l177;l100+l168-l176;l100+l168+l176];

l2 = A_max.^2;
l3 = J_max.^2;
l4 = t7.^2;
t2 = ((J_min.*l2-J_max.*l2-A_init.^2.*J_min+A_wayp.^2.*J_max+J_max.^3.*l4+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_wayp.*2.0-A_wayp.*l3.*t7.*2.0-J_min.*l3.*l4+A_wayp.*J_min.*J_max.*t7.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

t3 = -(A_max-A_wayp+J_max.*t7)./J_min;

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


