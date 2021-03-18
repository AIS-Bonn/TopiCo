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

function [t] = a_O_P(P_init,V_init,A_init,P_wayp,~,~,~,~,~,~,J_max,~) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = P_init.*6.0;
l5 = P_wayp.*6.0;
l6 = 1.0./J_max;
l10 = sqrt(3.0);
l7 = l6.^2;
l8 = l6.^3;
l9 = -l5;
l11 = A_init.*l6;
l12 = V_init.*l6.*2.0;
l13 = -l11;
l14 = A_init.*V_init.*l7.*3.0;
l15 = l2.*l7;
l16 = l3.*l8;
l17 = l4+l9;
l18 = -l14;
l19 = -l15;
l20 = -l16;
l21 = (l6.*l17)./2.0;
l22 = -l21;
l23 = l12+l19;
l25 = l16+l18+l21;
l24 = l23.^3;
l26 = l25.^2;
l27 = l24+l26;
l28 = sqrt(complex(l27));
l29 = l14+l20+l22+l28;
l30 = l29.^(1.0./3.0);
l31 = 1.0./l30;
l32 = l30./2.0;
l33 = -l32;
l34 = l23.*l31;
l35 = l34./2.0;
l36 = l30+l34;
l37 = l10.*l36.*5.0e-1i;
t1 = [l13+l30-l34;l13+l33+l35-l37;l13+l33+l35+l37];

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t3 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
