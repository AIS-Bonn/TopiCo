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

function [t] = abc_T_P(P_init,V_init,A_init,P_wayp,~,~,~,~,A_max,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 14:54:42
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_max.^2;
l5 = A_max.^3;
l6 = J_max.^2;
l7 = J_max.^3;
l8 = T.^2;
l9 = T.^3;
l10 = 1.0./J_min;
l19 = sqrt(3.0);
l23 = A_init.*A_max.*J_min.*J_max.*6.0;
l43 = A_init.*A_max.*J_min.*J_max.*T.*-6.0;
l11 = l10.^2;
l12 = l10.^3;
l13 = 1.0./l7;
l14 = 1.0./l6.^3;
l15 = J_min.*l3;
l16 = J_max.*l3;
l17 = J_min.*l5;
l18 = J_max.*l5;
l21 = P_init.*l7.*6.0;
l22 = P_wayp.*l7.*6.0;
l27 = -l23;
l28 = A_init.*J_min.*l4.*3.0;
l29 = A_max.*J_min.*l2.*3.0;
l30 = A_init.*J_max.*l4.*3.0;
l31 = A_max.*J_max.*l2.*3.0;
l32 = A_init.*J_min.*l6.*3.0;
l33 = J_min.*J_max.*l2.*3.0;
l34 = A_max.*J_min.*l6.*3.0;
l35 = J_min.*J_max.*l4.*3.0;
l36 = J_min.*T.*l7.*3.0;
l37 = T.*V_init.*l7.*6.0;
l39 = J_min.*l7.*l9;
l44 = A_init.*A_max.*T.*l6.*6.0;
l46 = A_init.*J_min.*T.*l6.*6.0;
l48 = A_max.*J_min.*T.*l6.*6.0;
l49 = J_min.*l7.*l8.*3.0;
l51 = T.*l2.*l6.*3.0;
l52 = T.*l4.*l6.*3.0;
l53 = A_max.*l7.*l8.*3.0;
l58 = A_max.*J_min.*l6.*l8.*-3.0;
l20 = l13.^3;
l24 = -l16;
l25 = -l17;
l26 = -l22;
l40 = -l29;
l41 = -l30;
l42 = -l34;
l45 = T.*l33;
l47 = T.*l35;
l50 = -l48;
l54 = l8.*l32;
l56 = -l51;
l57 = -l52;
l59 = l32+l36+l42;
l63 = l27+l33+l35+l46+l49+l50;
l66 = l15+l18+l21+l24+l25+l26+l28+l31+l37+l39+l40+l41+l43+l44+l45+l47+l53+l54+l56+l57+l58;
l60 = l59.^3;
l61 = (l10.*l13.*l59)./3.0;
l64 = (l11.*l14.*l59.*l63)./3.0;
l67 = l10.*l13.*l66;
l62 = l12.*l20.*l60.*(2.0./2.7e+1);
l65 = -l64;
l68 = l62+l65+l67;
l69 = l68.^(1.0./3.0);
l70 = l69./2.0;
l72 = l19.*l69.*5.0e-1i;
l71 = -l70;
t2 = [l61+l69;l61+l71-l72;l61+l71+l72];

t3 = T-t2+(A_init-A_max)./J_max;

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6];

t4 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t7 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

