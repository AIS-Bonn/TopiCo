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

function [t] = abcdeg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,~,V_max,V_min,A_max,~,J_max,J_min) %#codegen
% Generated on 04-Sep-2019 13:52:28
coder.inline('default');

l2 = -J_min;
l3 = -V_min;
l4 = sqrt(2.0);
l5 = sqrt(J_max);
l6 = l5.^3;
l7 = J_max+l2;
l8 = V_max+l3;
l9 = V_wayp+l3;
l10 = J_min.*l5;
l11 = sqrt(complex(l2));
l13 = sqrt(complex(l7));
l14 = sqrt(complex(l8));
l15 = sqrt(complex(l9));
l17 = J_min.*l4.*l15;
l18 = J_max.*l4.*l15;
l20 = l4.*l11.*l13.*l14;
t7 = [(l17-l18+l20)./(l6-l10);(l18+l20+l2.*l4.*l15)./(l6-l10)];

l2 = J_min.^2;
l3 = J_min.*J_max;
l4 = -J_max;
l5 = -V_max;
l6 = sqrt(2.0);
l7 = sqrt(J_max);
l8 = -l3;
l9 = J_min+l4;
l10 = V_min+l5;
l11 = l2+l8;
l13 = J_min.*l9.*l10;
l12 = 1.0./l11;
l14 = -l13;
l15 = sqrt(complex(l14));
l16 = l6.*l7.*l12.*l15;
t5 = [l16;l16];

l2 = A_init.^2;
l3 = A_max.^2;
l5 = J_min.^2;
l6 = J_min.^3;
l7 = J_max.^2;
l4 = l3.^2;
t4 = ((l4.*l5-l4.*l7-l2.^2.*l5.*3.0+A_init.^3.*A_max.*l5.*8.0-l2.*l3.*l5.*6.0-V_init.^2.*l5.*l7.*1.2e+1+V_max.^2.*l5.*l7.*1.2e+1+A_max.*l6.*l7.*t5.^3.*4.0+A_max.*J_max.^3.*l5.*t7.^3.*4.0+A_max.*P_init.*l5.*l7.*2.4e+1-A_max.*P_wayp.*l5.*l7.*2.4e+1+J_max.*V_init.*l2.*l5.*1.2e+1+J_max.*V_init.*l3.*l5.*1.2e+1-J_min.*V_max.*l3.*l7.*1.2e+1+A_max.*V_max.*l5.*l7.*t5.*2.4e+1+A_max.*V_max.*l5.*l7.*t7.*2.4e+1+A_max.*l6.*l7.*t5.*t7.^2.*1.2e+1+A_max.*l6.*l7.*t5.^2.*t7.*1.2e+1-A_init.*A_max.*J_max.*V_init.*l5.*2.4e+1).*(-1.0./2.4e+1))./(A_max.*V_max.*l5.*l7);

l2 = A_init.^2;
l3 = A_max.^2;
l4 = J_max.*V_init.*2.0;
l5 = J_max.*V_max.*2.0;
l6 = 1.0./A_max;
l7 = 1.0./J_min;
l8 = 1.0./J_max;
l9 = -l4;
l10 = -l3;
l11 = J_max.*l3.*l7;
l12 = l2+l5+l9+l10+l11;
l13 = (l6.*l8.*l12)./2.0;
t2 = [l13;l13];

l2 = 1.0./J_min;
l3 = A_max.*l2;
l4 = -l3;
t3 = [l4;l4];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t6 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


