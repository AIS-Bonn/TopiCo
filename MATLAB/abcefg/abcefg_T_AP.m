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

function [t] = abcefg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,~,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l6 = A_max.^2;
l7 = A_max.^3;
l8 = A_wayp.^2;
l9 = A_wayp.^3;
l10 = J_min.^2;
l11 = J_max.^2;
l12 = T.^2;
l13 = A_min.*J_min.*J_max;
l14 = A_max.*J_min.*J_max;
l15 = -A_max;
l16 = A_init.*A_min.*J_min.*6.0;
l17 = A_init.*A_max.*J_min.*6.0;
l18 = A_min.*A_max.*J_min.*6.0;
l19 = A_min.*A_max.*J_max.*6.0;
l20 = sqrt(3.0);
l21 = -l16;
l22 = -l19;
l23 = -l14;
l24 = A_min+l15;
l25 = J_max.*l4.*3.0;
l26 = J_max.*l6.*3.0;
l27 = J_min.*l6.*6.0;
l28 = T.*l13.*6.0;
l29 = T.*l14.*6.0;
l32 = l5.*l11;
l33 = l7.*l11;
l34 = l3.*l10.*4.0;
l35 = l5.*l10.*4.0;
l36 = l7.*l10.*4.0;
l37 = l9.*l10.*4.0;
l41 = A_min.*l6.*l11.*3.0;
l42 = A_max.*l4.*l11.*3.0;
l43 = P_init.*l10.*l11.*2.4e+1;
l44 = P_wayp.*l10.*l11.*2.4e+1;
l45 = A_init.*A_max.*J_max.*T.*l10.*2.4e+1;
l46 = A_init.*l6.*l10.*1.2e+1;
l47 = A_max.*l2.*l10.*1.2e+1;
l49 = A_min.*l8.*l10.*1.2e+1;
l50 = A_wayp.*l4.*l10.*1.2e+1;
l52 = J_max.*T.*l2.*l10.*1.2e+1;
l53 = J_max.*T.*l6.*l10.*1.2e+1;
l54 = T.*V_init.*l10.*l11.*2.4e+1;
l60 = A_max.*l10.*l11.*l12.*1.2e+1;
l30 = -l27;
l31 = -l28;
l38 = -l34;
l39 = -l35;
l40 = -l33;
l48 = -l42;
l51 = -l44;
l55 = l13+l23;
l56 = -l46;
l57 = -l49;
l58 = -l52;
l59 = -l53;
l61 = 1.0./l55;
l62 = l32+l36+l37+l38+l39+l40+l41+l43+l45+l47+l48+l50+l51+l54+l56+l57+l58+l59+l60;
l63 = l24.*l62;
l64 = -l63;
l65 = sqrt(complex(l64));
l66 = l20.*l65;
t2 = [l61.*(l17+l18+l21+l22+l25+l26+l29+l30+l31+l66).*(-1.0./6.0);(l61.*(l16-l17-l18+l19-l25-l26+l27+l28-l29+l66))./6.0];

l2 = -A_max;
t6 = (J_min.*(A_init+A_min-A_wayp+l2+J_max.*T)-J_max.*(A_min+l2+J_min.*t2))./(J_min.*J_max);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = -A_max;
l3 = 1.0./J_min;
l4 = A_min+l2;
l5 = l3.*l4;
t3 = [l5;l5];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
