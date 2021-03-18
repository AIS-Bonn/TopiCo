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

function [t] = aceg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_wayp.^2;
l6 = A_wayp.^3;
l8 = J_min.^2;
l9 = J_min.^3;
l11 = J_max.^2;
l12 = J_max.^3;
l13 = V_init.^2;
l14 = V_wayp.^2;
l17 = sqrt(3.0);
l18 = sqrt(6.0);
l30 = A_init.*J_min.*J_max.*V_init.*2.4e+1;
l31 = A_wayp.*J_min.*J_max.*V_wayp.*2.4e+1;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l20 = J_min.*l12.*2.0;
l21 = J_max.*l9.*4.0;
l22 = J_min.*l3.*8.0;
l23 = J_max.*l3.*8.0;
l24 = J_min.*l6.*8.0;
l25 = J_max.*l6.*8.0;
l26 = P_init.*l12.*2.4e+1;
l27 = P_wayp.*l12.*2.4e+1;
l28 = V_init.*l12.*1.2e+1;
l29 = V_wayp.*l12.*1.2e+1;
l35 = -l30;
l36 = J_min.*J_max.*l2.*1.2e+1;
l37 = J_min.*J_max.*l5.*1.2e+1;
l38 = J_max.*V_init.*l2.*1.2e+1;
l39 = A_init.*V_init.*l11.*2.4e+1;
l40 = J_max.*V_wayp.*l2.*1.2e+1;
l41 = J_max.*V_init.*l5.*1.2e+1;
l42 = J_max.*V_wayp.*l5.*1.2e+1;
l43 = A_wayp.*V_wayp.*l11.*2.4e+1;
l44 = l2.*l5.*6.0;
l45 = J_min.*P_init.*l11.*2.4e+1;
l46 = J_min.*P_wayp.*l11.*2.4e+1;
l47 = J_max.*V_init.*l8.*1.2e+1;
l48 = J_min.*V_init.*l11.*2.4e+1;
l49 = J_max.*V_wayp.*l8.*1.2e+1;
l50 = J_min.*V_wayp.*l11.*2.4e+1;
l51 = l2.*l8.*6.0;
l52 = l2.*l11.*6.0;
l53 = l5.*l8.*6.0;
l54 = l5.*l11.*6.0;
l55 = l8.*l11.*5.0;
l56 = V_init.*V_wayp.*l11.*2.4e+1;
l70 = l11.*l13.*1.2e+1;
l71 = l11.*l14.*1.2e+1;
l15 = l4.*3.0;
l16 = l7.*3.0;
l32 = -l23;
l33 = -l24;
l34 = -l26;
l57 = -l38;
l58 = -l42;
l59 = -l43;
l60 = -l44;
l61 = -l46;
l62 = -l48;
l63 = -l50;
l64 = -l51;
l65 = -l52;
l66 = -l53;
l67 = -l54;
l69 = -l56;
l73 = -1.0./(l10-l20-l21+l55);
l74 = 1.0./(l10-l20-l21+l55).^2;
l76 = l74.^2;
l78 = l15+l16+l40+l41+l57+l58+l60+l69+l70+l71;
l81 = l22+l25+l27+l31+l32+l33+l34+l35+l39+l45+l59+l61;
l84 = l28+l29+l36+l37+l47+l49+l62+l63+l64+l65+l66+l67;
l79 = l78.^2;
l80 = l78.^3;
l82 = l81.^2;
l85 = l84.^2;
l86 = l84.^3;
l88 = (l78.*-4.0)./(l10-l20-l21+l55);
l89 = (l78.*-1.2e+1)./(l10-l20-l21+l55);
l101 = l74.*l78.*l84.*(4.0./3.0);
l102 = l74.*l78.*l84.*7.2e+1;
l83 = l82.^2;
l87 = l85.^2;
l91 = l80.*1.0./(l10-l20-l21+l55).^3.*2.56e+2;
l92 = l74.*l82.*2.7e+1;
l94 = (l74.*l82)./2.0;
l95 = l74.*l85;
l97 = l86.*1.0./(l10-l20-l21+l55).^3.*2.0;
l100 = (l86.*1.0./(l10-l20-l21+l55).^3)./2.7e+1;
l105 = l76.*l79.*l85.*1.28e+2;
l107 = l82.*l86.*1.0./(l10-l20-l21+l55).^5.*4.0;
l108 = l76.*l78.*l82.*l84.*1.44e+2;
l93 = l76.*l83.*2.7e+1;
l98 = l95./3.0;
l104 = l78.*l87.*1.0./(l10-l20-l21+l55).^5.*1.6e+1;
l109 = l91+l93+l104+l105+l107+l108;
l110 = sqrt(complex(l109));
l111 = l17.*l110.*3.0;
l112 = (l17.*l110)./1.8e+1;
l113 = l92+l97+l102+l111;
l115 = l94+l100+l101+l112;
l114 = sqrt(complex(l113));
l116 = l115.^(1.0./3.0);
l118 = 1.0./l115.^(1.0./6.0);
l117 = l116.^2;
l121 = (l18.*l81.*l114.*-3.0)./(l10-l20-l21+l55);
l122 = (l18.*l81.*l114.*3.0)./(l10-l20-l21+l55);
l123 = (l84.*l116.*-2.0)./(l10-l20-l21+l55);
l124 = (l84.*l116.*-6.0)./(l10-l20-l21+l55);
l119 = l117.*3.0;
l120 = l117.*9.0;
l125 = l89+l95+l120+l124;
l126 = l88+l98+l119+l123;
l127 = sqrt(complex(l126));
l128 = 1.0./l125.^(1.0./4.0);
l130 = (l17.*l78.*l127.*1.2e+1)./(l10-l20-l21+l55);
l132 = (l17.*l73.*l85.*l127)./(l10-l20-l21+l55);
l134 = l17.*l117.*l127.*-9.0;
l135 = (l17.*l118.*l127)./6.0;
l137 = (l17.*l84.*l116.*l127.*-1.2e+1)./(l10-l20-l21+l55);
l136 = -l135;
l138 = l121+l130+l132+l134+l137;
l139 = l122+l130+l132+l134+l137;
l140 = sqrt(complex(l138));
l141 = sqrt(complex(l139));
l142 = (l118.*l128.*l140)./6.0;
l143 = (l118.*l128.*l141)./6.0;
t3 = [0.0;l135-l143;l135+l143;l136-l142;l136+l142];

l2 = t3.^2;
t7 = (J_max.*V_init.*2.0-J_max.*V_wayp.*2.0+J_min.^2.*l2-A_init.^2+A_wayp.^2-A_wayp.*J_min.*t3.*2.0+A_wayp.*J_max.*t3.*2.0-J_min.*J_max.*l2)./(J_max.^2.*t3.*2.0-J_min.*J_max.*t3.*2.0);

t1 = -(A_init-A_wayp+J_min.*t3+J_max.*t7)./J_max;

t6 = [0.0;0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


