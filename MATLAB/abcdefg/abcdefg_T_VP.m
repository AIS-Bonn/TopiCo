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

function [t] = abcdefg_T_VP(P_init,V_init,A_init,P_wayp,V_wayp,~,V_max,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 04-Sep-2019 15:47:08
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l9 = A_max.^3;
l11 = J_min.^2;
l12 = J_max.^2;
l13 = V_init.^2;
l14 = V_max.^2;
l15 = V_wayp.^2;
l16 = 1.0./A_max;
l17 = 1.0./J_min;
l18 = 1.0./J_max;
l23 = 1.0./V_max;
l24 = A_init.*A_min.*J_min.*2.0;
l25 = A_min.*A_max.*J_max.*2.0;
l26 = J_min.*J_max.*V_max.*2.0;
l30 = sqrt(3.0);
l31 = sqrt(6.0);
l32 = A_min.*J_min.*J_max.*T.*2.0;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l19 = 1.0./l12;
l20 = l18.^3;
l22 = l18.^5;
l27 = J_min.*l2;
l28 = J_max.*l8;
l29 = J_min.*l12;
l36 = (A_min.*l18)./3.0;
l42 = A_min.*A_max.*l3.*l11.*8.0;
l44 = A_init.*A_min.*A_max.*J_max.*V_init.*l11.*2.4e+1;
l48 = A_min.*A_max.*P_init.*l11.*l12.*2.4e+1;
l49 = A_min.*A_max.*P_wayp.*l11.*l12.*2.4e+1;
l50 = A_min.*J_max.*V_init.*l2.*l11.*1.2e+1;
l52 = A_min.*J_max.*V_max.*l2.*l11.*1.2e+1;
l53 = A_max.*J_max.*V_max.*l2.*l11.*1.2e+1;
l56 = A_min.*l2.*l8.*l11.*6.0;
l57 = A_min.*V_init.*V_max.*l11.*l12.*2.4e+1;
l58 = A_max.*V_max.*V_wayp.*l11.*l12.*2.4e+1;
l68 = A_min.*l11.*l12.*l13.*1.2e+1;
l69 = A_min.*l11.*l12.*l14.*1.2e+1;
l70 = A_max.*l11.*l12.*l14.*1.2e+1;
l71 = A_max.*l11.*l12.*l15.*1.2e+1;
l21 = l19.^2;
l33 = A_min.*l10.*l11;
l34 = A_min.*l10.*l12;
l35 = A_max.*l7.*l12;
l37 = A_min.*l4.*l11.*3.0;
l38 = -l36;
l39 = V_wayp.*l23.*l29;
l43 = V_max.*l9.*l29.*1.2e+1;
l45 = l5.*l19.*(2.0./3.0);
l46 = -l42;
l51 = A_min.*V_init.*l11.*l28.*1.2e+1;
l54 = A_min.*V_max.*l11.*l28.*1.2e+1;
l55 = A_min.*V_max.*l8.*l29.*2.4e+1;
l61 = -l48;
l62 = -l50;
l64 = -l57;
l67 = l6.*l20.*(8.0./2.7e+1);
l72 = -l71;
l40 = -l33;
l41 = -l35;
l47 = -l39;
l59 = l7.*l21.*(4.0./9.0);
l60 = (l7.*l21)./2.7e+1;
l63 = -l51;
l65 = -l59;
l66 = -l60;
l73 = l29+l47;
l96 = l34+l37+l40+l41+l43+l44+l46+l49+l52+l53+l54+l55+l56+l58+l61+l62+l63+l64+l68+l69+l70+l72;
l74 = V_max.*l17.*l20.*l73.*4.0;
l76 = A_min.*V_max.*l17.*l21.*l73.*(8.0./3.0);
l78 = V_max.*l5.*l17.*l22.*l73.*(4.0./9.0);
l79 = V_max.*l5.*l17.*l22.*l73.*(1.6e+1./3.0);
l97 = (l16.*l17.*l18.*l23.*l96)./1.2e+1;
l75 = -l74;
l77 = -l76;
l98 = -l97;
l80 = l45+l75;
l86 = l67+l77;
l99 = l24+l25+l26+l27+l28+l32+l98;
l81 = l80.^2;
l82 = l80.^3;
l88 = l86.^2;
l100 = V_max.*l17.*l20.*l99.*4.0;
l101 = V_max.*l17.*l20.*l99.*4.8e+1;
l83 = l81.^2;
l84 = l82.*2.0;
l87 = l82./2.7e+1;
l89 = l88.^2;
l91 = l88.*2.7e+1;
l93 = l88./2.0;
l94 = l82.*l88.*4.0;
l102 = l66+l78+l100;
l85 = -l84;
l90 = -l87;
l92 = l89.*2.7e+1;
l95 = -l94;
l103 = l102.^2;
l104 = l102.^3;
l107 = l80.*l102.*(4.0./3.0);
l108 = l80.*l102.*7.2e+1;
l109 = l83.*l102.*1.6e+1;
l112 = l80.*l88.*l102.*1.44e+2;
l105 = l104.*2.56e+2;
l110 = -l109;
l111 = l81.*l103.*1.28e+2;
l106 = -l105;
l113 = l92+l95+l106+l110+l111+l112;
l114 = sqrt(complex(l113));
l115 = l30.*l114.*3.0;
l116 = (l30.*l114)./1.8e+1;
l117 = l85+l91+l108+l115;
l119 = l90+l93+l107+l116;
l118 = sqrt(complex(l117));
l120 = l119.^(1.0./3.0);
l122 = 1.0./l119.^(1.0./6.0);
l121 = l120.^2;
l124 = l80.*l120.*6.0;
l125 = l31.*l86.*l118.*3.0;
l123 = l121.*9.0;
l126 = -l125;
l127 = l65+l79+l81+l101+l123+l124;
l128 = sqrt(complex(l127));
l129 = 1.0./l127.^(1.0./4.0);
l130 = l81.*l128;
l132 = l102.*l128.*1.2e+1;
l135 = l121.*l128.*-9.0;
l136 = (l122.*l128)./6.0;
l138 = l80.*l120.*l128.*1.2e+1;
l131 = -l130;
l133 = -l132;
l137 = -l136;
l139 = l125+l131+l133+l135+l138;
l140 = l126+l131+l133+l135+l138;
l141 = sqrt(complex(l139));
l142 = sqrt(complex(l140));
l143 = (l122.*l129.*l141)./6.0;
l144 = (l122.*l129.*l142)./6.0;
t7 = [l38+l137-l143;l38+l137+l143;l38+l136-l144;l38+l136+l144];

l2 = A_init.^2;
l3 = A_max.^2;
l4 = J_max.*V_init.*2.0;
l5 = J_max.*V_max.*2.0;
l6 = 1.0./A_max;
l7 = 1.0./J_min;
l8 = 1.0./J_max;
l9 = -l4;
l10 = -l3;
l11 = J_max.*l3.*l7;
l12 = l2+l5+l9+l10+l11;
l13 = (l6.*l8.*l12)./2.0;
t2 = [l13;l13;l13;l13];

l2 = A_init.^2;
l3 = A_min.^2;
l4 = A_max.^2;
l5 = A_max.^3;
l7 = J_min.^2;
l8 = J_max.^2;
l9 = J_max.^3;
l10 = t2.^2;
l6 = l4.^2;
t4 = -(l6.*l7.*-3.0-l6.*l8.*3.0-l2.^2.*l7.*3.0+l3.^2.*l8+J_min.*J_max.*l6.*6.0+l7.*l8.^2.*t7.^4.*3.0+A_min.*l5.*l7.*4.0+A_min.*l5.*l8.*8.0+A_init.^3.*A_min.*l7.*8.0+l2.*l4.*l7.*6.0-l3.*l4.*l8.*6.0-V_init.^2.*l7.*l8.*1.2e+1+V_wayp.^2.*l7.*l8.*1.2e+1+J_min.*l5.*l8.*t2.*1.2e+1-J_max.*l5.*l7.*t2.*1.2e+1-l4.*l7.*l8.*l10.*1.2e+1+A_min.*l7.*l9.*t7.^3.*4.0-V_wayp.*l7.*l9.*t7.^2.*1.2e+1-A_min.*J_min.*J_max.*l5.*1.2e+1-A_min.*A_max.*l2.*l7.*1.2e+1+A_min.*P_init.*l7.*l8.*2.4e+1-A_min.*P_wayp.*l7.*l8.*2.4e+1-J_min.*J_max.*l2.*l3.*6.0-J_min.*J_max.*l2.*l4.*6.0+J_min.*J_max.*l3.*l4.*6.0+J_min.*V_init.*l3.*l8.*1.2e+1+J_max.*V_init.*l2.*l7.*1.2e+1+J_min.*V_init.*l4.*l8.*1.2e+1-J_max.*V_init.*l4.*l7.*1.2e+1+A_min.*A_max.*l7.*l8.*l10.*1.2e+1-A_min.*J_max.*l2.*l7.*t2.*1.2e+1-A_min.*J_min.*l4.*l8.*t2.*2.4e+1+A_min.*J_max.*l4.*l7.*t2.*1.2e+1+A_max.*J_min.*l3.*l8.*t2.*1.2e+1+A_max.*J_max.*l2.*l7.*t2.*1.2e+1+A_min.*V_init.*l7.*l8.*t2.*2.4e+1-A_max.*V_init.*l7.*l8.*t2.*2.4e+1+A_min.*A_max.*J_min.*J_max.*l2.*1.2e+1-A_init.*A_min.*J_max.*V_init.*l7.*2.4e+1-A_min.*A_max.*J_min.*V_init.*l8.*2.4e+1+A_min.*A_max.*J_max.*V_init.*l7.*2.4e+1)./(A_min.*J_max.*l2.*l7.*-1.2e+1-A_min.*J_min.*l4.*l8.*1.2e+1+A_min.*J_max.*l4.*l7.*1.2e+1+A_min.*V_init.*l7.*l8.*2.4e+1+A_min.*A_max.*l7.*l8.*t2.*2.4e+1);

l2 = A_init.^2;
l3 = A_max.^2;
l4 = 1.0./J_min;
t6 = (l4.*(J_min.*l2.*2.0+J_max.*l3-J_min.*(l2+l3+J_max.*V_init.*2.0+J_max.^2.*t7.^2+A_max.*J_max.*t2.*2.0+A_min.*J_max.*t7.*2.0+A_min.^2.*J_max.*l4)+J_min.*J_max.*V_wayp.*2.0))./(A_min.*J_max.*2.0);

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3;l3;l3];

l2 = 1.0./J_min;
l3 = A_max.*l2;
l4 = -l3;
t3 = [l4;l4;l4;l4];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t = [t1, t2, t3, t4, t5, t6, t7];

end

