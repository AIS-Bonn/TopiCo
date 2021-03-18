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

function [t] = acde_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 15:48:59
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = J_min.^2;
l5 = J_min.^3;
l7 = J_max.^2;
l8 = J_min.^5;
l9 = J_max.^3;
l11 = J_max.*V_init.*2.0;
l12 = J_max.*V_max.*2.0;
l13 = -J_max;
l15 = sqrt(3.0);
l6 = l4.^2;
l10 = l7.^2;
l14 = -l11;
l16 = J_min+l13;
l18 = l7.*l8;
l30 = J_max.*l3.*l5.*4.0;
l33 = P_init.*l5.*l9.*1.2e+1;
l34 = P_wayp.*l5.*l9.*1.2e+1;
l35 = A_init.*V_init.*l4.*l9.*6.0;
l36 = A_init.*V_max.*l4.*l9.*6.0;
l40 = A_init.*V_init.*l5.*l7.*1.2e+1;
l42 = A_init.*V_max.*l5.*l7.*1.2e+1;
l43 = l3.*l4.*l7.*2.0;
l46 = T.*V_max.*l5.*l9.*1.2e+1;
l17 = l5.*l10;
l19 = l3.*l6.*2.0;
l20 = l6.*l9.*2.0;
l21 = A_init.*J_max.*V_init.*l6.*6.0;
l22 = A_init.*J_max.*V_max.*l6.*6.0;
l23 = P_init.*l4.*l10.*6.0;
l24 = P_init.*l6.*l7.*6.0;
l25 = P_wayp.*l4.*l10.*6.0;
l26 = P_wayp.*l6.*l7.*6.0;
l37 = T.*V_max.*l4.*l10.*6.0;
l38 = T.*V_max.*l6.*l7.*6.0;
l49 = l2+l12+l14;
l28 = -l20;
l50 = J_min.*l16.*l49;
l51 = l17+l18+l28;
l52 = sqrt(complex(l50));
l53 = l52.^3;
l56 = 1.0./l51;
l57 = J_max.*V_init.*l5.*l52.*6.0;
l58 = J_max.*V_max.*l5.*l52.*6.0;
l60 = l2.*l5.*l52.*3.0;
l62 = J_max.*l2.*l4.*l52.*3.0;
l63 = V_init.*l4.*l7.*l52.*6.0;
l64 = V_max.*l4.*l7.*l52.*6.0;
l54 = J_min.*l53;
l55 = J_max.*l53;
l67 = -l56.*(l19-l21+l22+l23+l24-l25-l26-l30-l33+l34-l35+l36+l37+l38+l40-l42+l43-l46-l54-l55-l57+l58+l60-l62+l63-l64);
l68 = l67.^(1.0./3.0);
l69 = l68./2.0;
l71 = l15.*l68.*5.0e-1i;
l70 = -l69;
t5 = [l70-l71;l70+l71;l68];

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
l3 = J_min.^2;
l4 = J_max.^2;
l5 = t3.^2;
l6 = t3.^3;
t4 = (P_init.*l4.*6.0-P_wayp.*l4.*6.0-J_min.^3.*l6+A_init.^3.*2.0-A_init.*J_max.*V_init.*6.0-J_min.*l4.*l6.*2.0+J_max.*l3.*l6.*3.0+J_min.*l2.*t3.*3.0-J_max.*l2.*t3.*3.0-J_max.*l2.*t5.*3.0+V_init.*l4.*t3.*6.0+V_init.*l4.*t5.*6.0+J_min.*l4.*t5.^3-J_min.*l4.*l5.*t5.*3.0+J_max.*l3.*l5.*t5.*3.0-J_min.*J_max.*V_init.*t3.*6.0)./(J_max.*l2.*3.0-V_init.*l4.*6.0+J_min.*l4.*l5.*3.0-J_max.*l3.*l5.*3.0);

t1 = -(A_init+J_min.*t3)./J_max;

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


