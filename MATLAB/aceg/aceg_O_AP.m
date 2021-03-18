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

function [t] = aceg_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_wayp.^2;
l6 = A_wayp.^3;
l8 = J_min.^2;
l9 = J_min.^3;
l11 = J_max.^2;
l12 = J_min.^5;
l13 = J_max.^3;
l15 = V_init.^2;
l16 = V_max.^2;
l17 = sqrt(3.0);
l18 = sqrt(6.0);
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l14 = l8.^3;
l20 = J_max.*l12.*4.0;
l30 = l9.*l13.*2.0;
l33 = l3.*l9.*8.0;
l35 = l6.*l9.*8.0;
l37 = A_init.*J_max.*V_init.*l9.*2.4e+1;
l38 = A_wayp.*J_max.*V_max.*l9.*2.4e+1;
l39 = J_min.*l5.*l13.*6.0;
l40 = P_init.*l8.*l13.*2.4e+1;
l41 = P_init.*l9.*l11.*2.4e+1;
l42 = P_wayp.*l8.*l13.*2.4e+1;
l43 = P_wayp.*l9.*l11.*2.4e+1;
l44 = V_init.*l8.*l13.*1.2e+1;
l45 = V_init.*l9.*l11.*2.4e+1;
l46 = V_max.*l8.*l13.*1.2e+1;
l47 = V_max.*l9.*l11.*2.4e+1;
l48 = J_min.*J_max.*l2.*l5.*6.0;
l51 = J_max.*l3.*l8.*8.0;
l52 = J_max.*l2.*l9.*1.2e+1;
l53 = J_min.*l6.*l11.*1.2e+1;
l54 = J_max.*l5.*l9.*1.8e+1;
l55 = J_max.*l6.*l8.*2.0e+1;
l60 = J_max.*V_init.*l2.*l8.*1.2e+1;
l61 = A_init.*V_init.*l8.*l11.*2.4e+1;
l62 = J_max.*V_max.*l2.*l8.*1.2e+1;
l63 = J_min.*V_init.*l5.*l11.*1.2e+1;
l64 = J_max.*V_init.*l5.*l8.*1.2e+1;
l65 = J_min.*V_max.*l5.*l11.*1.2e+1;
l66 = J_max.*V_max.*l5.*l8.*1.2e+1;
l67 = A_wayp.*V_max.*l8.*l11.*2.4e+1;
l68 = l2.*l5.*l8.*6.0;
l69 = l2.*l8.*l11.*6.0;
l70 = V_init.*V_max.*l8.*l11.*2.4e+1;
l78 = l5.*l8.*l11.*1.8e+1;
l80 = l8.*l11.*l15.*1.2e+1;
l81 = l8.*l11.*l16.*1.2e+1;
l21 = J_min.*J_max.*l7.*6.0;
l23 = J_max.*V_init.*l10.*1.2e+1;
l24 = J_max.*V_max.*l10.*1.2e+1;
l25 = l4.*l8.*3.0;
l26 = l2.*l10.*6.0;
l27 = l7.*l8.*3.0;
l28 = l7.*l11.*3.0;
l29 = l5.*l10.*6.0;
l31 = l10.*l11.*5.0;
l49 = -l33;
l50 = -l38;
l56 = -l41;
l57 = -l42;
l58 = -l45;
l59 = -l47;
l71 = -l55;
l72 = -l60;
l73 = -l61;
l74 = -l63;
l75 = -l66;
l76 = -l68;
l77 = -l69;
l79 = -l70;
l82 = -l78;
l22 = -l21;
l32 = -l26;
l34 = -l29;
l84 = -1.0./(l14-l20-l30+l31);
l85 = 1.0./(l14-l20-l30+l31).^2;
l93 = l35+l37+l40+l43+l49+l50+l51+l53+l56+l57+l67+l71+l73;
l87 = l85.^2;
l89 = l23+l24+l32+l34+l39+l44+l46+l52+l54+l58+l59+l77+l82;
l94 = l93.^2;
l96 = l22+l25+l27+l28+l48+l62+l64+l65+l72+l74+l75+l76+l79+l80+l81;
l90 = l89.^2;
l91 = l89.^3;
l95 = l94.^2;
l97 = l96.^2;
l98 = l96.^3;
l105 = l85.*l94.*2.7e+1;
l107 = (l85.*l94)./2.0;
l108 = (l96.*-4.0)./(l14-l20-l30+l31);
l109 = (l96.*-1.2e+1)./(l14-l20-l30+l31);
l114 = l85.*l89.*l96.*(4.0./3.0);
l115 = l85.*l89.*l96.*7.2e+1;
l119 = l87.*l89.*l94.*l96.*1.44e+2;
l92 = l90.^2;
l99 = l85.*l90;
l101 = l91.*1.0./(l14-l20-l30+l31).^3.*2.0;
l104 = (l91.*1.0./(l14-l20-l30+l31).^3)./2.7e+1;
l106 = l87.*l95.*2.7e+1;
l111 = l98.*1.0./(l14-l20-l30+l31).^3.*2.56e+2;
l113 = l91.*l94.*1.0./(l14-l20-l30+l31).^5.*4.0;
l118 = l87.*l90.*l97.*1.28e+2;
l102 = l99./3.0;
l117 = l92.*l96.*1.0./(l14-l20-l30+l31).^5.*1.6e+1;
l120 = l106+l111+l113+l117+l118+l119;
l121 = sqrt(complex(l120));
l122 = l17.*l121.*3.0;
l123 = (l17.*l121)./1.8e+1;
l124 = l101+l105+l115+l122;
l126 = l104+l107+l114+l123;
l125 = sqrt(complex(l124));
l127 = l126.^(1.0./3.0);
l129 = 1.0./l126.^(1.0./6.0);
l128 = l127.^2;
l132 = (l89.*l127.*-2.0)./(l14-l20-l30+l31);
l133 = (l89.*l127.*-6.0)./(l14-l20-l30+l31);
l134 = (l18.*l93.*l125.*-3.0)./(l14-l20-l30+l31);
l135 = (l18.*l93.*l125.*3.0)./(l14-l20-l30+l31);
l130 = l128.*3.0;
l131 = l128.*9.0;
l136 = l99+l109+l131+l133;
l137 = l102+l108+l130+l132;
l138 = sqrt(complex(l137));
l139 = 1.0./l136.^(1.0./4.0);
l141 = (l17.*l84.*l90.*l138)./(l14-l20-l30+l31);
l143 = (l17.*l96.*l138.*1.2e+1)./(l14-l20-l30+l31);
l145 = l17.*l128.*l138.*-9.0;
l146 = (l17.*l129.*l138)./6.0;
l148 = (l17.*l89.*l127.*l138.*-1.2e+1)./(l14-l20-l30+l31);
l147 = -l146;
l149 = l134+l141+l143+l145+l148;
l150 = l135+l141+l143+l145+l148;
l151 = sqrt(complex(l149));
l152 = sqrt(complex(l150));
l153 = (l129.*l139.*l151)./6.0;
l154 = (l129.*l139.*l152)./6.0;
t3 = [l147-l154;l147+l154;l146-l153;l146+l153];

l2 = J_min.*t3;
l3 = -A_wayp;
l4 = A_init+l2+l3;
t7 = (J_min.*l4.^2+J_max.*l2.^2+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_max.*2.0-A_init.*J_min.*l4.*2.0+A_wayp.*J_max.*l3+J_max.*l2.*(A_wayp-l2).*2.0)./(J_max.^2.*l2.*2.0+A_init.*J_min.*J_max.*2.0-A_wayp.*J_min.*J_max.*2.0-J_min.*J_max.*l4.*2.0);

t1 = -(A_init-A_wayp+J_min.*t3+J_max.*t7)./J_max;

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


