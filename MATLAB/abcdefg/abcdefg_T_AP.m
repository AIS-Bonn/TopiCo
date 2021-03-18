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

function [t] = abcdefg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l7 = A_max.^2;
l9 = A_wayp.^2;
l10 = A_wayp.^3;
l11 = J_min.^2;
l12 = J_max.^2;
l13 = V_init.^2;
l14 = V_max.^2;
l15 = 1.0./A_min;
l16 = 1.0./A_max;
l17 = 1.0./J_min;
l18 = 1.0./J_max;
l19 = sqrt(3.0);
l20 = A_init.*A_min.*A_max.*J_min.*6.0;
l21 = A_min.*J_min.*J_max.*V_init.*6.0;
l22 = A_min.*J_min.*J_max.*V_max.*6.0;
l29 = A_min.*A_max.*J_min.*J_max.*T.*6.0;
l4 = l2.^2;
l8 = l7.^2;
l23 = A_min.*J_min.*l2.*3.0;
l24 = A_min.*J_min.*l7.*3.0;
l25 = A_min.*J_max.*l7.*3.0;
l26 = A_max.*J_max.*l5.*3.0;
l31 = A_max.*l6.*l12;
l33 = A_max.*l6.*l11.*4.0;
l34 = A_max.*l10.*l11.*4.0;
l35 = A_init.*A_max.*J_max.*V_init.*l11.*2.4e+1;
l36 = A_init.*A_max.*J_max.*V_max.*l11.*2.4e+1;
l37 = A_max.*l3.*l11.*8.0;
l41 = A_min.*A_max.*l9.*l11.*1.2e+1;
l42 = A_max.*A_wayp.*l5.*l11.*1.2e+1;
l43 = A_max.*P_init.*l11.*l12.*2.4e+1;
l44 = A_max.*P_wayp.*l11.*l12.*2.4e+1;
l45 = J_max.*V_init.*l2.*l11.*1.2e+1;
l46 = J_max.*V_max.*l2.*l11.*1.2e+1;
l47 = J_max.*V_init.*l7.*l11.*1.2e+1;
l48 = J_max.*V_max.*l7.*l11.*1.2e+1;
l49 = l2.*l7.*l11.*6.0;
l50 = V_init.*V_max.*l11.*l12.*2.4e+1;
l57 = l11.*l12.*l13.*1.2e+1;
l58 = l11.*l12.*l14.*1.2e+1;
l59 = A_max.*T.*V_max.*l11.*l12.*2.4e+1;
l27 = l8.*l11;
l28 = l8.*l12;
l30 = l4.*l11.*3.0;
l62 = -A_min.*A_max.*(l27-l28-l30+l31-l33+l34-l35+l36+l37-l41+l42+l43-l44+l45-l46+l47-l48-l49+l50-l57-l58+l59);
l63 = sqrt(complex(l62));
l64 = l19.*l63;
t4 = [(l15.*l16.*l17.*l18.*(l20+l21-l22-l23-l24+l25-l26+l29+l64))./6.0;l15.*l16.*l17.*l18.*(-l20-l21+l22+l23+l24-l25+l26-l29+l64).*(-1.0./6.0)];

l2 = A_max.^2;
t6 = ((J_min.*l2-J_max.*l2+A_init.^2.*J_min-A_init.*A_max.*J_min.*2.0-A_min.*A_max.*J_min.*2.0+A_min.*A_max.*J_max.*2.0+A_max.*A_wayp.*J_min.*2.0-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_max.*2.0-A_max.*J_min.*J_max.*T.*2.0+A_max.*J_min.*J_max.*t4.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

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

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

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

t = [t1, t2, t3, t4, t5, t6, t7];

end


