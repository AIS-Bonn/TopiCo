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

function [t] = acef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,~,~,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l7 = J_min.^2;
l8 = J_min.^3;
l10 = J_max.^2;
l11 = V_init.^2;
l12 = V_wayp.^2;
l15 = sqrt(3.0);
l16 = sqrt(6.0);
l22 = A_init.*A_min.*J_max.*V_init.*2.4e+1;
l4 = l2.^2;
l6 = l5.^2;
l9 = l7.^2;
l18 = J_max.*l8.*6.0;
l19 = A_min.*l3.*8.0;
l20 = A_min.*l8.*8.0;
l23 = J_min.*J_max.*l2.*6.0;
l24 = A_min.*J_min.*l10.*4.0;
l25 = J_min.*J_max.*l5.*6.0;
l26 = -l22;
l28 = A_min.*J_max.*l7.*1.2e+1;
l29 = A_min.*P_init.*l10.*2.4e+1;
l30 = A_min.*P_wayp.*l10.*2.4e+1;
l31 = J_max.*V_init.*l2.*1.2e+1;
l32 = J_max.*V_init.*l5.*1.2e+1;
l33 = l2.*l5.*6.0;
l34 = J_min.*V_init.*l10.*1.2e+1;
l35 = J_max.*V_init.*l7.*1.2e+1;
l36 = l2.*l7.*6.0;
l37 = l5.*l7.*6.0;
l38 = l7.*l10.*3.0;
l44 = l10.*l11.*1.2e+1;
l45 = l10.*l12.*1.2e+1;
l13 = l4.*3.0;
l14 = l9.*3.0;
l21 = -l18;
l39 = -l28;
l40 = -l30;
l41 = -l33;
l46 = -l44;
l17 = -l13;
l47 = l14+l21+l38;
l52 = l20+l24+l39;
l48 = 1.0./l47;
l53 = l52.^2;
l54 = l52.^3;
l64 = l6+l17+l19+l26+l29+l31+l32+l40+l41+l45+l46;
l49 = l48.^2;
l50 = l48.^3;
l55 = l53.^2;
l57 = (l48.*l52)./4.0;
l63 = -l48.*(l23-l25-l34+l35-l36+l37);
l69 = l48.*l64;
l51 = l49.^2;
l58 = l49.*l53.*(3.0./8.0);
l59 = (l50.*l54)./8.0;
l65 = l49.*l52.*(l23-l25-l34+l35-l36+l37).*(-1.0./2.0);
l67 = l50.*l53.*(l23-l25-l34+l35-l36+l37).*(3.0./4.0);
l68 = l50.*l53.*(l23-l25-l34+l35-l36+l37).*(-1.0./1.6e+1);
l70 = l69.*1.2e+1;
l60 = l51.*l55.*(3.0./2.56e+2);
l61 = l51.*l55.*(9.0./6.4e+1);
l71 = -l70;
l72 = l58+l63;
l80 = l59+l65;
l62 = -l61;
l73 = l72.^2;
l74 = l72.^3;
l81 = l80.^2;
l86 = l60+l68+l69;
l75 = l73.^2;
l76 = l74.*2.0;
l78 = l74./2.7e+1;
l82 = l81.^2;
l83 = l81.*2.7e+1;
l85 = l81./2.0;
l87 = l86.^2;
l88 = l86.^3;
l90 = l74.*l81.*4.0;
l92 = l72.*l86.*(4.0./3.0);
l93 = l72.*l86.*7.2e+1;
l98 = l72.*l81.*l86.*1.44e+2;
l77 = -l76;
l79 = -l78;
l84 = l82.*2.7e+1;
l89 = l88.*2.56e+2;
l91 = -l90;
l94 = -l92;
l95 = -l93;
l96 = l75.*l86.*1.6e+1;
l97 = l73.*l87.*1.28e+2;
l99 = -l98;
l100 = l84+l89+l91+l96+l97+l99;
l101 = sqrt(complex(l100));
l102 = l15.*l101.*3.0;
l103 = (l15.*l101)./1.8e+1;
l104 = l77+l83+l95+l102;
l106 = l79+l85+l94+l103;
l105 = sqrt(complex(l104));
l107 = l106.^(1.0./3.0);
l109 = 1.0./l106.^(1.0./6.0);
l108 = l107.^2;
l111 = l72.*l107.*6.0;
l112 = l16.*l80.*l105.*3.0;
l110 = l108.*9.0;
l113 = -l112;
l114 = l62+l67+l71+l73+l110+l111;
l115 = sqrt(complex(l114));
l116 = 1.0./l114.^(1.0./4.0);
l117 = l73.*l115;
l119 = l86.*l115.*1.2e+1;
l121 = l108.*l115.*-9.0;
l122 = (l109.*l115)./6.0;
l124 = l72.*l107.*l115.*1.2e+1;
l118 = -l117;
l123 = -l122;
l125 = l112+l118+l119+l121+l124;
l126 = l113+l118+l119+l121+l124;
l127 = sqrt(complex(l125));
l128 = sqrt(complex(l126));
l129 = (l109.*l116.*l127)./6.0;
l130 = (l109.*l116.*l128)./6.0;
t3 = [l57+l123-l130;l57+l123+l130;l57+l122-l129;l57+l122+l129];

l2 = t3.^2;
t6 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0+J_min.^2.*l2-A_init.^2+A_min.^2-A_min.*J_min.*t3.*2.0+A_min.*J_max.*t3.*2.0-J_min.*J_max.*l2).*(-1.0./2.0))./(A_min.*J_max);

t1 = -(A_init-A_min+J_min.*t3)./J_max;

t7 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


