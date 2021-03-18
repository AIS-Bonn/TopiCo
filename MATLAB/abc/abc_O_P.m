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

function [t] = abc_O_P(P_init,V_init,A_init,P_wayp,~,~,V_max,~,A_max,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l6 = A_max.^3;
l8 = J_min.^2;
l9 = J_max.^2;
l10 = V_init.^2;
l11 = V_max.^2;
l12 = 1.0./A_max;
l13 = 1.0./J_min;
l18 = sqrt(3.0);
l4 = l2.^2;
l7 = l5.^2;
l14 = 1.0./l8;
l15 = l13.^3;
l17 = 1.0./l9;
l19 = A_max.*l13;
l27 = A_init.*A_max.*J_max.*V_init.*l8.*2.4e+1;
l29 = A_max.*l3.*l8.*8.0;
l30 = J_min.*l6.*l9.*1.2e+1;
l32 = A_max.*P_init.*l8.*l9.*2.4e+1;
l33 = A_max.*P_wayp.*l8.*l9.*2.4e+1;
l34 = J_max.*V_init.*l2.*l8.*1.2e+1;
l35 = J_max.*V_init.*l5.*l8.*1.2e+1;
l36 = J_min.*V_max.*l5.*l9.*1.2e+1;
l37 = A_max.*V_max.*l8.*l9.*2.4e+1;
l38 = l2.*l5.*l8.*6.0;
l41 = l8.*l9.*l10.*1.2e+1;
l42 = l8.*l9.*l11.*1.2e+1;
l16 = l14.^2;
l20 = l7.*l8;
l21 = -l19;
l22 = l4.*l8.*3.0;
l23 = l5.*l14;
l24 = l6.*l15;
l25 = l7.*l9.*3.0;
l31 = -l27;
l39 = -l33;
l40 = -l38;
l43 = -l41;
l44 = l30+l37;
l26 = -l22;
l28 = -l24;
l45 = (l16.*l17.*l44)./8.0;
l47 = (l12.*l15.*l17.*l44)./1.2e+1;
l46 = -l45;
l48 = -l47;
l52 = l20+l25+l26+l29+l31+l32+l34+l35+l36+l39+l40+l42+l43;
l49 = l23+l48;
l53 = (l12.*l15.*l17.*l52)./8.0;
l50 = l49.^3;
l54 = -l53;
l55 = l24+l46+l53;
l51 = -l50;
l56 = l55.^2;
l57 = l51+l56;
l58 = sqrt(complex(l57));
l59 = l28+l45+l54+l58;
l60 = l59.^(1.0./3.0);
l61 = 1.0./l60;
l62 = l60./2.0;
l63 = -l62;
l64 = l49.*l61;
l65 = -l64;
l66 = l64./2.0;
l67 = -l66;
l68 = l60+l65;
l69 = l18.*l68.*5.0e-1i;
t3 = [l21+l60+l64;l21+l63+l67-l69;l21+l63+l67+l69];

l2 = -A_max;
l3 = A_init+l2;
t2 = ((J_min.*l3.^2-J_max.*(A_max+J_min.*t3).^2+J_min.^2.*J_max.*t3.^2+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_max.*2.0-A_init.*J_min.*l3.*2.0+A_max.*J_min.*J_max.*t3.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
