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

function [t] = acdeg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 11:55:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_wayp.^2;
l5 = A_wayp.^3;
l6 = J_min.^2;
l7 = J_min.^3;
l9 = J_max.^2;
l10 = J_max.^3;
l12 = J_max.^5;
l14 = J_max.*V_init.*2.0;
l15 = J_max.*V_max.*2.0;
l16 = -J_min;
l18 = sqrt(3.0);
l8 = l6.^2;
l11 = l9.^2;
l13 = l9.^3;
l17 = -l14;
l19 = J_max+l16;
l20 = l16.^(5.0./2.0);
l21 = l16.^(7.0./2.0);
l22 = l16.^(9.0./2.0);
l23 = l16.^(1.1e+1./2.0);
l24 = l19.^(3.0./2.0);
l25 = l2+l15+l17;
l26 = J_min.*l13.*l24;
l27 = A_wayp.*J_min.*l12.*l24.*3.0;
l28 = sqrt(complex(l25));
l30 = l3.*l8.*l24.*2.0;
l31 = l8.*l10.*l24.*2.0;
l32 = l6.*l12.*l24.*4.0;
l33 = l7.*l11.*l24.*5.0;
l34 = A_init.*J_max.*V_init.*l8.*l24.*6.0;
l35 = A_init.*J_max.*V_max.*l8.*l24.*6.0;
l36 = J_min.*l5.*l10.*l24;
l41 = J_max.*l3.*l7.*l24.*2.0;
l42 = A_wayp.*l8.*l9.*l24.*3.0;
l43 = J_min.*l4.*l11.*l24.*3.0;
l44 = P_init.*l7.*l10.*l24.*6.0;
l45 = P_init.*l8.*l9.*l24.*6.0;
l46 = P_wayp.*l7.*l10.*l24.*6.0;
l47 = P_wayp.*l8.*l9.*l24.*6.0;
l48 = l5.*l6.*l9.*l24;
l49 = T.*V_max.*l7.*l10.*l24.*6.0;
l50 = T.*V_max.*l8.*l9.*l24.*6.0;
l52 = A_wayp.*l6.*l11.*l24.*9.0;
l53 = A_wayp.*l7.*l10.*l24.*9.0;
l56 = A_init.*V_init.*l7.*l9.*l24.*6.0;
l57 = A_init.*V_max.*l7.*l9.*l24.*6.0;
l58 = l4.*l7.*l9.*l24.*3.0;
l59 = l4.*l6.*l10.*l24.*6.0;
l29 = l28.^3;
l37 = -l30;
l38 = -l31;
l39 = -l32;
l40 = -l35;
l51 = -l42;
l54 = -l45;
l55 = -l46;
l60 = -l50;
l61 = -l52;
l62 = -l56;
l63 = -l48;
l64 = -l59;
l71 = J_max.*V_init.*l23.*l28.*6.0;
l72 = J_max.*V_max.*l23.*l28.*6.0;
l74 = l2.*l23.*l28.*3.0;
l75 = V_init.*l10.*l21.*l28.*6.0;
l76 = V_max.*l10.*l21.*l28.*6.0;
l77 = J_max.*l2.*l22.*l28.*6.0;
l79 = l2.*l9.*l21.*l28.*3.0;
l80 = V_init.*l9.*l22.*l28.*1.2e+1;
l81 = V_max.*l9.*l22.*l28.*1.2e+1;
l65 = l23.*l29;
l66 = J_max.*l22.*l29;
l67 = l10.*l20.*l29;
l68 = l9.*l21.*l29;
l73 = -l71;
l78 = -l75;
l82 = -l80;
l83 = l43+l58+l64;
l84 = l26+l33+l38+l39;
l88 = l27+l51+l53+l61;
l69 = -l65;
l70 = -l66;
l85 = 1.0./l84;
l89 = l88.^2;
l90 = l88.^3;
l86 = l85.^2;
l87 = l85.^3;
l91 = (l83.*l85)./3.0;
l93 = (l85.*l88)./3.0;
l101 = l34+l36+l37+l40+l41+l44+l47+l49+l54+l55+l57+l60+l62+l63+l67+l68+l69+l70+l72+l73+l74+l76+l77+l78+l79+l81+l82;
l94 = (l86.*l89)./9.0;
l95 = (l87.*l90)./2.7e+1;
l96 = (l83.*l86.*l88)./6.0;
l102 = (l85.*l101)./2.0;
l97 = -l96;
l100 = (l91-l94).^3;
l103 = l95+l97+l102;
l104 = l103.^2;
l105 = l100+l104;
l106 = sqrt(complex(l105));
l107 = l103+l106;
l108 = l107.^(1.0./3.0);
l109 = 1.0./l108;
l111 = l108./2.0;
l112 = -l111;
l113 = -l109.*(l91-l94);
l115 = (l109.*(l91-l94))./2.0;
l117 = l18.*(l108+l109.*(l91-l94)).*-5.0e-1i;
t7 = [l93+l108+l113;l93+l112+l115+l18.*(l108+l109.*(l91-l94)).*5.0e-1i;l93+l112+l115+l117];

l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_min.*J_max;
l5 = J_max.*V_init.*2.0;
l6 = J_max.*V_max.*2.0;
l7 = -J_max;
l8 = -l5;
l9 = -l4;
l10 = J_min+l7;
l11 = l3+l9;
l13 = l2+l6+l8;
l12 = 1.0./l11;
l14 = J_min.*l10.*l13;
l15 = sqrt(complex(l14));
l16 = l12.*l15;
t3 = [l16;l16;l16];

l2 = A_init.^2;
l3 = A_wayp.^2;
l4 = J_min.^2;
l5 = J_min.^3;
l7 = J_max.^2;
l8 = J_max.^3;
l10 = t3.^2;
l11 = t3.^3;
l12 = t7.^2;
l13 = t7.^3;
l6 = l4.^2;
l9 = l7.^2;
t4 = (A_init.^3.*l4.*2.0+A_wayp.^3.*l7-J_min.^5.*l11-J_max.^5.*l13+A_wayp.*l9.*l12.*3.0+J_max.*l6.*l11.*3.0+J_min.*l9.*l13.*3.0+P_init.*l4.*l7.*6.0-P_wayp.*l4.*l7.*6.0-l5.*l7.*l11.*2.0-l4.*l8.*l13.*2.0+l2.*l5.*t3.*3.0-l3.*l8.*t7.*3.0-J_max.*l2.*l4.*t3.*3.0-J_max.*l2.*l4.*t7.*3.0+J_min.*l2.*l7.*t7.*3.0+J_min.*l3.*l7.*t7.*3.0+J_max.*l6.*l10.*t7.*3.0+V_init.*l4.*l7.*t3.*6.0+V_init.*l4.*l7.*t7.*6.0+l4.*l8.*l10.*t7.*3.0-l5.*l7.*l10.*t7.*6.0-A_wayp.*J_min.*J_max.*l2.*3.0-A_init.*J_max.*V_init.*l4.*6.0+A_wayp.*J_min.*V_init.*l7.*6.0+A_wayp.*J_max.*l5.*l10.*3.0-A_wayp.*J_min.*l8.*l12.*6.0-J_max.*V_init.*l5.*t3.*6.0-J_min.*V_init.*l8.*t7.*6.0-A_wayp.*l4.*l7.*l10.*3.0+A_wayp.*l4.*l7.*l12.*3.0)./(J_max.*l2.*l4.*3.0-J_max.*l6.*l10.*3.0-V_init.*l4.*l7.*6.0+l5.*l7.*l10.*3.0);

t5 = (A_wayp-J_max.*t7)./J_min;

t1 = -(A_init+J_min.*t3)./J_max;

t6 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


