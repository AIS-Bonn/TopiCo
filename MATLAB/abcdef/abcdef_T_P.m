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

function [t] = abcdef_T_P(P_init,V_init,A_init,P_wayp,~,~,V_max,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 14:36:33
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l7 = A_max.^2;
l9 = J_min.^2;
l10 = J_max.^2;
l11 = V_init.^2;
l12 = V_max.^2;
l13 = 1.0./A_min;
l14 = 1.0./A_max;
l15 = 1.0./J_min;
l16 = 1.0./J_max;
l17 = sqrt(3.0);
l4 = l2.^2;
l8 = l7.^2;
l18 = A_max.*J_max.*l5.*3.0;
l22 = A_max.*l6.*l10;
l24 = A_init.*A_max.*J_max.*V_init.*l9.*2.4e+1;
l25 = A_init.*A_max.*J_max.*V_max.*l9.*2.4e+1;
l26 = A_max.*l3.*l9.*8.0;
l29 = A_max.*P_init.*l9.*l10.*2.4e+1;
l30 = A_max.*P_wayp.*l9.*l10.*2.4e+1;
l31 = J_max.*V_init.*l2.*l9.*1.2e+1;
l32 = J_max.*V_max.*l2.*l9.*1.2e+1;
l33 = J_max.*V_init.*l7.*l9.*1.2e+1;
l34 = J_max.*V_max.*l7.*l9.*1.2e+1;
l35 = l2.*l7.*l9.*6.0;
l36 = V_init.*V_max.*l9.*l10.*2.4e+1;
l42 = l9.*l10.*l11.*1.2e+1;
l43 = l9.*l10.*l12.*1.2e+1;
l44 = A_max.*T.*V_max.*l9.*l10.*2.4e+1;
l19 = l8.*l9;
l20 = l8.*l10;
l21 = l4.*l9.*3.0;
l47 = -A_min.*A_max.*(l19-l20-l21+l22-l24+l25+l26+l29-l30+l31-l32+l33-l34-l35+l36-l42-l43+l44);
l48 = sqrt(complex(l47));
l49 = l17.*l48;
t6 = [l13.*l14.*l15.*l16.*(l18-l49).*(-1.0./6.0);l13.*l14.*l15.*l16.*(l18+l49).*(-1.0./6.0)];

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
t4 = (l2.*(A_init.*J_min-A_max.*J_min+A_max.*J_max+J_min.*J_max.*T-J_min.*J_max.*(t2+t6+A_min.*l2)))./J_max;

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

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

t7 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


