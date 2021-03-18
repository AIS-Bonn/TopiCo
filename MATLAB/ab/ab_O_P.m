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

function [t] = ab_O_P(P_init,V_init,A_init,P_wayp,~,~,~,~,A_max,~,J_max,~) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l7 = J_max.^2;
l8 = V_init.^2;
l9 = J_max.*V_init.*6.0;
l10 = 1.0./A_max;
l11 = 1.0./J_max;
l15 = sqrt(3.0);
l19 = A_init.*A_max.*J_max.*V_init.*2.4e+1;
l4 = l2.^2;
l6 = l5.^2;
l12 = l2.*3.0;
l14 = l5.*3.0;
l18 = A_max.*l3.*8.0;
l21 = A_max.*P_init.*l7.*2.4e+1;
l22 = A_max.*P_wayp.*l7.*2.4e+1;
l23 = J_max.*V_init.*l2.*1.2e+1;
l24 = J_max.*V_init.*l5.*1.2e+1;
l25 = l2.*l5.*6.0;
l29 = l7.*l8.*1.2e+1;
l13 = l4.*3.0;
l16 = -l12;
l17 = -l6;
l20 = -l18;
l26 = -l21;
l27 = -l23;
l28 = -l24;
l30 = l13+l17+l19+l20+l22+l25+l26+l27+l28+l29;
l31 = sqrt(complex(l30));
l32 = l15.*l31;
t2 = [l10.*l11.*(l9+l14+l16+l32).*(-1.0./6.0);l10.*l11.*(l9+l14+l16-l32).*(-1.0./6.0)];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t7 = [0.0;0.0];

t6 = [0.0;0.0];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t3 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
