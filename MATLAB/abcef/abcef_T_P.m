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

function [t] = abcef_T_P(P_init,V_init,A_init,P_wayp,~,~,~,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 13:28:28
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l6 = A_max.^2;
l7 = A_max.^3;
l8 = J_min.^2;
l9 = J_max.^2;
l10 = T.^2;
l11 = A_min.*J_min.*J_max;
l12 = A_max.*J_min.*J_max;
l13 = -A_max;
l14 = A_min.*A_max.*J_max.*6.0;
l15 = sqrt(3.0);
l16 = -l12;
l17 = A_min+l13;
l18 = J_max.*l4.*3.0;
l19 = J_max.*l6.*3.0;
l20 = l5.*l9;
l21 = l7.*l9;
l22 = l3.*l8.*4.0;
l23 = l7.*l8.*4.0;
l26 = A_min.*l6.*l9.*3.0;
l27 = A_max.*l4.*l9.*3.0;
l28 = P_init.*l8.*l9.*2.4e+1;
l29 = P_wayp.*l8.*l9.*2.4e+1;
l30 = A_init.*A_max.*J_max.*T.*l8.*2.4e+1;
l31 = A_init.*l6.*l8.*1.2e+1;
l32 = A_max.*l2.*l8.*1.2e+1;
l35 = J_max.*T.*l2.*l8.*1.2e+1;
l36 = J_max.*T.*l6.*l8.*1.2e+1;
l37 = T.*V_init.*l8.*l9.*2.4e+1;
l42 = A_max.*l8.*l9.*l10.*1.2e+1;
l24 = -l22;
l25 = -l21;
l33 = -l27;
l34 = -l29;
l38 = l11+l16;
l39 = -l31;
l40 = -l35;
l41 = -l36;
l43 = 1.0./l38;
l44 = l20+l23+l24+l25+l26+l28+l30+l32+l33+l34+l37+l39+l40+l41+l42;
l45 = l17.*l44;
l46 = -l45;
l47 = sqrt(complex(l46));
l48 = l15.*l47;
t6 = [(l43.*(l14-l18-l19+l48))./6.0;l43.*(-l14+l18+l19+l48).*(-1.0./6.0)];

l2 = -A_max;
t2 = (J_min.*(A_init+l2+J_max.*T)-J_max.*(A_min+l2+J_min.*t6))./(J_min.*J_max);

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

t7 = [0.0;0.0];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
