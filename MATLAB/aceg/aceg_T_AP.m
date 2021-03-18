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

function [t] = aceg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_wayp.^2;
l5 = A_wayp.^3;
l6 = J_min.^2;
l7 = J_max.^2;
l8 = J_max.^3;
l9 = T.^2;
l10 = T.^3;
l11 = A_init.*J_min;
l12 = A_init.*J_max;
l13 = A_wayp.*J_min;
l14 = A_wayp.*J_max;
l15 = J_max.*T;
l17 = -A_wayp;
l18 = A_init.*A_wayp.*6.0;
l24 = sqrt(3.0);
l32 = J_min.*J_max.*P_init.*4.8e+1;
l33 = J_min.*J_max.*P_wayp.*4.8e+1;
l16 = J_min.*l15;
l19 = l2.*3.0;
l20 = l4.*3.0;
l21 = T.*l12.*6.0;
l22 = T.*l14.*6.0;
l23 = T.*l7;
l25 = -l18;
l28 = -l3;
l36 = A_init.*l4.*-3.0;
l37 = -l32;
l38 = P_init.*l6.*2.4e+1;
l39 = P_init.*l7.*2.4e+1;
l40 = P_wayp.*l6.*2.4e+1;
l41 = P_wayp.*l7.*2.4e+1;
l48 = l2.*l15.*-3.0;
l49 = l4.*l15.*-3.0;
l50 = T.*V_init.*l6.*2.4e+1;
l52 = l7.*l9.*3.0;
l53 = l8.*l10.*3.0;
l54 = J_max.*l6.*l10.*4.0;
l55 = A_init+l15+l17;
l56 = J_max.*l9.*l11.*2.4e+1;
l58 = J_min.*l7.*l10.*8.0;
l60 = A_init.*l7.*l9.*9.0;
l61 = A_init.*l6.*l9.*1.2e+1;
l29 = -l22;
l31 = A_wayp.*l19;
l35 = A_wayp.*l21;
l42 = V_init.*l16.*4.8e+1;
l45 = -l40;
l46 = -l41;
l51 = V_init.*l23.*2.4e+1;
l57 = A_wayp.*l52;
l59 = -l56;
l62 = -l58;
l47 = -l42;
l65 = l5+l28+l31+l33+l35+l36+l37+l38+l39+l45+l46+l47+l48+l49+l50+l51+l53+l54+l57+l59+l60+l61+l62;
l66 = l55.*l65;
l67 = sqrt(complex(l66));
l68 = l24.*l67;
t7 = [(l19+l20+l21+l25+l29+l52+l68)./(l11.*6.0-l12.*6.0-l13.*6.0+l14.*6.0+l16.*6.0-l23.*6.0);(l19+l20+l21+l25+l29+l52-l68)./(l11.*6.0-l12.*6.0-l13.*6.0+l14.*6.0+l16.*6.0-l23.*6.0)];

l2 = J_max.*T;
l3 = -A_wayp;
l4 = -J_max;
l5 = J_min+l4;
l6 = A_init+l2+l3;
l7 = 1.0./l5;
l8 = l6.*l7;
l9 = -l8;
t3 = [l9;l9];

t1 = -(A_init-A_wayp+J_min.*t3+J_max.*t7)./J_max;

t6 = [0.0;0.0];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


