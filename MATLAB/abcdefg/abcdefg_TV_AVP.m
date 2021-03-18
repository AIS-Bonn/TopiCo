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

function [t] = abcdefg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l9 = A_max.^3;
l11 = A_wayp.^2;
l12 = A_wayp.^3;
l14 = J_min.^2;
l15 = J_max.^2;
l16 = T.^2;
l17 = V_init.^2;
l18 = V_wayp.^2;
l19 = 1.0./A_min;
l20 = 1.0./A_max;
l21 = 1.0./J_min;
l22 = 1.0./J_max;
l23 = sqrt(3.0);
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l13 = l11.^2;
l24 = A_min.*J_max.*l8.*3.0;
l25 = A_max.*J_max.*l5.*3.0;
l32 = A_min.*l9.*l15;
l33 = A_max.*l6.*l15;
l36 = A_min.*l3.*l14.*4.0;
l37 = A_min.*l9.*l14.*4.0;
l38 = A_max.*l6.*l14.*4.0;
l39 = A_max.*l12.*l14.*4.0;
l40 = A_init.*A_min.*A_max.*A_wayp.*l14.*2.4e+1;
l41 = A_init.*A_max.*J_max.*V_init.*l14.*2.4e+1;
l42 = A_min.*A_wayp.*J_max.*V_init.*l14.*2.4e+1;
l43 = A_init.*A_max.*J_max.*V_wayp.*l14.*2.4e+1;
l44 = A_min.*A_wayp.*J_max.*V_wayp.*l14.*2.4e+1;
l46 = A_max.*l3.*l14.*8.0;
l50 = A_min.*l12.*l14.*8.0;
l54 = A_init.*A_min.*l8.*l14.*1.2e+1;
l55 = A_init.*A_max.*l5.*l14.*1.2e+1;
l56 = A_min.*A_max.*l2.*l14.*1.2e+1;
l57 = A_min.*A_wayp.*l2.*l14.*1.2e+1;
l58 = A_init.*A_max.*l11.*l14.*1.2e+1;
l59 = A_min.*A_max.*l11.*l14.*1.2e+1;
l60 = A_min.*A_wayp.*l8.*l14.*1.2e+1;
l61 = A_max.*A_wayp.*l5.*l14.*1.2e+1;
l62 = A_min.*P_init.*l14.*l15.*2.4e+1;
l63 = A_max.*P_init.*l14.*l15.*2.4e+1;
l64 = A_min.*P_wayp.*l14.*l15.*2.4e+1;
l65 = A_max.*P_wayp.*l14.*l15.*2.4e+1;
l66 = J_max.*V_init.*l2.*l14.*1.2e+1;
l67 = J_max.*V_init.*l5.*l14.*1.2e+1;
l68 = J_max.*V_init.*l8.*l14.*1.2e+1;
l69 = J_max.*V_wayp.*l2.*l14.*1.2e+1;
l70 = J_max.*V_init.*l11.*l14.*1.2e+1;
l71 = J_max.*V_wayp.*l5.*l14.*1.2e+1;
l72 = J_max.*V_wayp.*l8.*l14.*1.2e+1;
l73 = J_max.*V_wayp.*l11.*l14.*1.2e+1;
l74 = l2.*l5.*l14.*6.0;
l75 = l2.*l8.*l14.*6.0;
l76 = l5.*l8.*l14.*6.0;
l77 = l2.*l11.*l14.*6.0;
l78 = l5.*l11.*l14.*6.0;
l79 = l8.*l11.*l14.*6.0;
l80 = V_init.*V_wayp.*l14.*l15.*2.4e+1;
l81 = A_init.*A_min.*A_max.*J_max.*T.*l14.*2.4e+1;
l82 = A_min.*A_max.*A_wayp.*J_max.*T.*l14.*2.4e+1;
l98 = l14.*l15.*l17.*1.2e+1;
l99 = l14.*l15.*l18.*1.2e+1;
l101 = A_min.*J_max.*T.*l2.*l14.*1.2e+1;
l102 = A_min.*J_max.*T.*l8.*l14.*1.2e+1;
l103 = A_max.*J_max.*T.*l5.*l14.*1.2e+1;
l104 = A_max.*J_max.*T.*l11.*l14.*1.2e+1;
l105 = A_min.*T.*V_init.*l14.*l15.*2.4e+1;
l106 = A_max.*T.*V_wayp.*l14.*l15.*2.4e+1;
l110 = A_min.*A_max.*l14.*l15.*l16.*1.2e+1;
l26 = l7.*l14;
l27 = l7.*l15;
l28 = l10.*l14;
l29 = l10.*l15;
l30 = l4.*l14.*3.0;
l31 = l13.*l14.*3.0;
l45 = -l36;
l47 = -l32;
l48 = -l33;
l49 = -l39;
l51 = -l40;
l52 = -l42;
l53 = -l43;
l83 = -l46;
l84 = -l50;
l85 = -l54;
l86 = -l61;
l87 = -l63;
l88 = -l64;
l89 = -l66;
l90 = -l68;
l91 = -l71;
l92 = -l73;
l93 = -l74;
l94 = -l76;
l95 = -l77;
l96 = -l79;
l97 = -l80;
l100 = -l82;
l107 = -l101;
l108 = -l102;
l109 = -l106;
l34 = -l26;
l35 = -l28;
l111 = l27+l29+l30+l31+l34+l35+l37+l38+l41+l44+l45+l47+l48+l49+l51+l52+l53+l55+l56+l57+l58+l59+l60+l62+l65+l67+l69+l70+l72+l75+l78+l81+l83+l84+l85+l86+l87+l88+l89+l90+l91+l92+l93+l94+l95+l96+l97+l98+l99+l100+l103+l104+l105+l107+l108+l109+l110;
l112 = A_min.*A_max.*l111;
l113 = sqrt(complex(l112));
l114 = l23.*l113;
t4 = [l19.*l20.*l21.*l22.*(-l24+l25+l114).*(-1.0./6.0);(l19.*l20.*l21.*l22.*(l24-l25+l114))./6.0];

l2 = A_min.^2;
l3 = A_max.^2;
t6 = (J_min.*l2+J_min.*l3-J_max.*l2-J_max.*l3+A_init.^2.*J_min-A_wayp.^2.*J_min-A_init.*A_max.*J_min.*2.0-A_min.*A_max.*J_min.*2.0+A_min.*A_max.*J_max.*2.0+A_max.*A_wayp.*J_min.*2.0-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_wayp.*2.0-A_max.*J_min.*J_max.*T.*2.0+A_max.*J_min.*J_max.*t4.*2.0)./(A_min.*J_min.*J_max.*2.0-A_max.*J_min.*J_max.*2.0);

l2 = 1.0./J_min;
l3 = 1.0./J_max;
t2 = l2.*l3.*(A_init.*J_min-A_max.*J_min+A_max.*J_max+J_min.*J_max.*T-J_min.*J_max.*(t4+t6+A_min.*l2-l3.*(A_min-A_wayp)));

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

l2 = 1.0./J_min;
l3 = A_max.*l2;
l4 = -l3;
t3 = [l4;l4];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t = [t1, t2, t3, t4, t5, t6, t7];

end
