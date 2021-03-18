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

function [t] = acef_T_P(P_init,V_init,A_init,P_wayp,~,~,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 13:28:28
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l6 = J_min.^2;
l7 = J_max.^2;
l8 = T.^2;
l9 = T.^3;
l10 = A_init.*A_min.*J_min.*6.0;
l12 = sqrt(3.0);
l17 = J_min.*J_max.*P_init.*1.2e+1;
l18 = J_min.*J_max.*P_wayp.*1.2e+1;
l28 = A_init.*A_min.*J_max.*T.*6.0;
l29 = A_init.*J_min.*J_max.*T.*6.0;
l30 = A_min.*J_min.*J_max.*T.*6.0;
l38 = J_min.*J_max.*T.*V_init.*1.2e+1;
l11 = J_min.*l7;
l13 = -l3;
l14 = -l10;
l15 = A_init.*l4.*3.0;
l16 = A_min.*l2.*3.0;
l19 = A_init.*l6.*3.0;
l20 = J_min.*l2.*3.0;
l21 = A_min.*l6.*3.0;
l22 = J_min.*l4.*3.0;
l23 = J_max.*l6.*2.0;
l24 = P_init.*l6.*6.0;
l25 = P_init.*l7.*6.0;
l26 = P_wayp.*l6.*6.0;
l27 = P_wayp.*l7.*6.0;
l32 = -l17;
l37 = -l30;
l39 = J_max.*T.*l2.*3.0;
l40 = J_max.*T.*l4.*3.0;
l41 = J_max.*T.*l6.*3.0;
l42 = T.*V_init.*l6.*6.0;
l43 = T.*V_init.*l7.*6.0;
l44 = J_max.*l6.*l9;
l45 = -l38;
l48 = A_init.*J_min.*J_max.*l8.*6.0;
l53 = A_min.*l7.*l8.*3.0;
l31 = -l15;
l33 = -l21;
l34 = -l23;
l35 = -l26;
l36 = -l27;
l46 = -l39;
l47 = -l40;
l49 = l8.*l11.*3.0;
l50 = l9.*l11.*2.0;
l51 = -l48;
l52 = l8.*l19;
l54 = -l50;
l55 = l11+l34;
l59 = l19+l33+l41;
l62 = l14+l20+l22+l29+l37+l49;
l56 = 1.0./l55;
l60 = l59.^2;
l61 = l59.^3;
l74 = l5+l13+l16+l18+l24+l25+l28+l31+l32+l35+l36+l42+l43+l44+l45+l46+l47+l51+l52+l53+l54;
l57 = l56.^2;
l58 = l56.^3;
l63 = (l56.*l59)./3.0;
l68 = (l56.*l62)./3.0;
l75 = (l56.*l74)./2.0;
l64 = -l63;
l65 = (l57.*l60)./9.0;
l66 = (l58.*l61)./2.7e+1;
l69 = (l57.*l59.*l62)./6.0;
l76 = -l75;
l67 = -l66;
l70 = -l69;
l71 = l65+l68;
l77 = l66+l69+l76;
l72 = l71.^3;
l78 = l77.^2;
l73 = -l72;
l79 = l73+l78;
l80 = sqrt(complex(l79));
l81 = l67+l70+l75+l80;
l82 = l81.^(1.0./3.0);
l83 = 1.0./l82;
l85 = l82./2.0;
l86 = -l85;
l87 = l71.*l83;
l88 = l87./2.0;
l91 = l12.*(l82-l87).*-5.0e-1i;
l89 = -l88;
t6 = [l64+l82+l87;l64+l86+l89+l12.*(l82-l87).*5.0e-1i;l64+l86+l89+l91];

l2 = -J_max;
t3 = -(A_init-A_min+l2.*t6+J_max.*T)./(J_min+l2);

t1 = -(A_init-A_min+J_min.*t3)./J_max;

t7 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


