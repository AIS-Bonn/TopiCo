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

function [t] = abcdefg_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,V_min,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 14:12:58
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l10 = J_min.^2;
l11 = J_max.^2;
l12 = V_init.^2;
l13 = V_min.^2;
l14 = V_max.^2;
l15 = 1.0./A_min;
l16 = 1.0./A_max;
l17 = 1.0./J_min;
l19 = 1.0./J_max;
l23 = 1.0./V_max;
l24 = A_init.*A_max.*J_min.*2.0;
l25 = J_min.*J_max.*V_init.*2.0;
l26 = J_min.*J_max.*V_min.*2.0;
l27 = J_min.*J_max.*V_max.*2.0;
l30 = sqrt(3.0);
l33 = A_max.*J_min.*J_max.*T.*2.0;
l4 = l2.^2;
l7 = l5.^2;
l9 = l8.^2;
l18 = 1.0./l10;
l20 = 1.0./l11;
l21 = l19.^3;
l28 = J_min.*l5;
l29 = J_max.*l8;
l31 = -l27;
l32 = A_min.*l19;
l46 = A_min.*A_max.*l3.*l10.*8.0;
l47 = A_max.*J_max.*l6.*l10.*1.2e+1;
l48 = J_max.*V_max.*l6.*l10.*1.2e+1;
l49 = A_init.*A_min.*A_max.*J_max.*V_init.*l10.*2.4e+1;
l51 = A_min.*A_max.*P_init.*l10.*l11.*2.4e+1;
l52 = A_min.*A_max.*P_wayp.*l10.*l11.*2.4e+1;
l53 = A_min.*J_max.*V_init.*l2.*l10.*1.2e+1;
l55 = A_min.*J_max.*V_max.*l2.*l10.*1.2e+1;
l56 = A_max.*J_max.*V_min.*l5.*l10.*1.2e+1;
l57 = A_min.*J_min.*V_max.*l8.*l11.*1.2e+1;
l59 = A_max.*J_max.*V_max.*l5.*l10.*1.2e+1;
l60 = A_min.*A_max.*V_min.*l10.*l11.*2.4e+1;
l61 = A_min.*A_max.*V_max.*l10.*l11.*2.4e+1;
l62 = A_min.*l2.*l8.*l10.*6.0;
l63 = A_min.*V_min.*V_max.*l10.*l11.*2.4e+1;
l64 = A_max.*V_min.*V_max.*l10.*l11.*2.4e+1;
l70 = A_min.*l10.*l11.*l12.*1.2e+1;
l71 = A_min.*l10.*l11.*l14.*1.2e+1;
l72 = A_max.*l10.*l11.*l13.*1.2e+1;
l73 = A_max.*l10.*l11.*l14.*1.2e+1;
l22 = l20.^2;
l34 = -l32;
l35 = l5.*l20;
l36 = l6.*l21;
l37 = A_min.*l9.*l10;
l38 = A_min.*l9.*l11;
l39 = A_max.*l7.*l11;
l40 = A_min.*l4.*l10.*3.0;
l41 = A_max.*l7.*l10.*3.0;
l50 = -l46;
l54 = A_min.*V_init.*l10.*l29.*1.2e+1;
l58 = A_min.*V_max.*l10.*l29.*1.2e+1;
l65 = -l51;
l66 = -l53;
l68 = -l56;
l69 = -l61;
l74 = -l71;
l75 = -l72;
l76 = -l73;
l42 = -l36;
l43 = -l37;
l44 = -l41;
l45 = -l39;
l67 = -l54;
l77 = l47+l60+l69;
l78 = (l16.*l18.*l22.*l77)./8.0;
l80 = (l15.*l16.*l18.*l21.*l77)./1.2e+1;
l85 = l38+l40+l43+l44+l45+l48+l49+l50+l52+l55+l57+l58+l59+l62+l63+l64+l65+l66+l67+l68+l70+l74+l75+l76;
l79 = -l78;
l81 = -l80;
l86 = (l15.*l17.*l19.*l23.*l85)./1.2e+1;
l82 = l35+l81;
l87 = -l86;
l83 = l82.^3;
l88 = l24+l25+l26+l28+l29+l31+l33+l87;
l84 = -l83;
l89 = V_max.*l16.*l17.*l20.*l88.*(3.0./2.0);
l90 = -l89;
l91 = l36+l79+l89;
l92 = l91.^2;
l93 = l84+l92;
l94 = sqrt(complex(l93));
l95 = l42+l78+l90+l94;
l96 = l95.^(1.0./3.0);
l97 = 1.0./l96;
l98 = l96./2.0;
l99 = -l98;
l100 = l82.*l97;
l101 = -l100;
l102 = l100./2.0;
l103 = -l102;
l104 = l96+l101;
l105 = l30.*l104.*5.0e-1i;
t7 = [l34+l96+l100;l34+l99+l103-l105;l34+l99+l103+l105];

l2 = A_min.^2;
l3 = J_min.*V_max.*2.0;
l4 = J_max.*V_min.*2.0;
l5 = 1.0./A_min;
l6 = 1.0./J_min;
l7 = 1.0./J_max;
l8 = l2+l3;
l9 = l2+l4;
l10 = J_min.*l9;
l11 = J_max.*l8;
l12 = -l11;
l13 = l10+l12;
l14 = (l5.*l6.*l7.*l13)./2.0;
t6 = [l14;l14;l14];

l2 = A_init.^2;
l3 = A_min.^2;
l4 = A_min.^3;
l6 = A_max.^2;
l8 = J_min.^2;
l9 = J_max.^2;
l10 = t6.^2;
l5 = l3.^2;
l7 = l6.^2;
t4 = (l5.*l8.*3.0+l5.*l9.*3.0+l7.*l8-l7.*l9-l2.^2.*l8.*3.0-J_min.*J_max.*l5.*6.0-A_max.*l4.*l9.*8.0+A_init.^3.*A_max.*l8.*8.0-l2.*l6.*l8.*6.0+l3.*l6.*l9.*6.0-V_init.^2.*l8.*l9.*1.2e+1+V_min.^2.*l8.*l9.*1.2e+1+J_min.*l4.*l9.*t6.*1.2e+1-J_max.*l4.*l8.*t6.*1.2e+1+l3.*l8.*l9.*l10.*1.2e+1+A_max.*J_max.^3.*l8.*t7.^3.*4.0+A_max.*J_min.*J_max.*l4.*1.2e+1+A_max.*P_init.*l8.*l9.*2.4e+1-A_max.*P_wayp.*l8.*l9.*2.4e+1-J_min.*J_max.*l3.*l6.*6.0+J_max.*V_init.*l2.*l8.*1.2e+1+J_max.*V_init.*l6.*l8.*1.2e+1-J_min.*V_min.*l3.*l9.*1.2e+1+J_max.*V_min.*l3.*l8.*1.2e+1-J_min.*V_min.*l6.*l9.*1.2e+1-A_min.*A_max.*l8.*l9.*l10.*1.2e+1-A_max.*J_min.*l3.*l9.*t6.*2.4e+1+A_min.*J_min.*l6.*l9.*t6.*1.2e+1+A_max.*J_max.*l3.*l8.*t6.*1.2e+1+A_max.*J_max.*l3.*l8.*t7.*1.2e+1-A_min.*V_min.*l8.*l9.*t6.*2.4e+1+A_max.*V_min.*l8.*l9.*t6.*2.4e+1+A_max.*V_min.*l8.*l9.*t7.*2.4e+1+A_min.*A_max.*l8.*l9.*t7.^2.*1.2e+1-A_init.*A_max.*J_max.*V_init.*l8.*2.4e+1+A_min.*A_max.*J_min.*V_min.*l9.*2.4e+1)./(A_max.*J_min.*l3.*l9.*1.2e+1-A_max.*J_max.*l3.*l8.*1.2e+1-A_max.*V_min.*l8.*l9.*2.4e+1+A_min.*A_max.*l8.*l9.*t6.*2.4e+1);

l2 = A_min.^2;
l3 = A_max.^2;
t2 = (J_min.*l2-J_min.*l3-J_max.*l2+J_max.*l3+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_min.*J_min.*J_max.*t6.*2.0)./(A_max.*J_min.*J_max.*2.0);

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3;l3];

l2 = 1.0./J_min;
l3 = A_max.*l2;
l4 = -l3;
t3 = [l4;l4;l4];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t = [t1, t2, t3, t4, t5, t6, t7];

end
