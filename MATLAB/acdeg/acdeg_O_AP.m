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

function [t] = acdeg_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,~,J_max,J_min) %#codegen
% Generated on 05-Sep-2019 11:33:59
coder.inline('default');

l2 = A_init.^2;
l3 = J_max.^2;
l4 = A_init.*J_min;
l5 = A_init.*J_max;
l6 = J_min.*J_max;
l7 = J_max.*V_init.*2.0;
l8 = J_max.*V_max.*2.0;
l9 = -J_max;
l10 = -l7;
l11 = -l6;
l12 = J_min+l9;
l13 = l3+l11;
l15 = l2+l8+l10;
l14 = 1.0./l13;
l16 = J_min.*l12.*l15;
l17 = sqrt(complex(l16));
t1 = [l14.*(l4-l5+l17);-l14.*(-l4+l5+l17)];

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_wayp.^2;
l5 = A_wayp.^3;
l6 = J_min.^2;
l7 = J_min.^3;
l9 = J_max.^2;
l10 = J_max.^3;
l12 = J_max.^5;
l14 = t1.^2;
l15 = t1.^3;
l16 = -J_min;
l17 = J_min.*J_max.*V_init.*2.0;
l18 = J_min.*J_max.*V_max.*2.0;
l24 = A_init.*J_min.*J_max.*t1.*2.0;
l8 = l6.^2;
l11 = l9.^2;
l13 = l9.^3;
l19 = J_max.*l2;
l21 = J_max.*l4;
l22 = -l17;
l23 = J_max+l16;
l25 = l4.*l16;
l26 = -l24;
l27 = A_init.*l9.*t1.*2.0;
l28 = l10.*l14;
l29 = J_min.*l9.*l14;
l30 = l9.*l14.*l16;
l31 = l23.^(3.0./2.0);
l32 = l23.^(5.0./2.0);
l33 = l18+l19+l21+l22+l25+l26+l27+l28+l30;
l34 = sqrt(complex(l33));
l35 = l34.^3;
t4 = -(l5.*l8.*-2.0+l3.*l11.*2.0+J_max.^7.*l15.*2.0+A_init.*l13.*l14.*6.0-A_init.*l7.*l28.*1.5e+1-J_min.*l3.*l10.*4.0+J_max.*l5.*l7.*7.0+J_min.*l5.*l10.*3.0-J_min.*l13.*l15.*7.0+J_min.*l31.*l35.*2.0-J_max.*l31.*l35+P_init.*l6.*l11.*6.0-P_init.*l7.*l10.*1.2e+1+P_init.*l8.*l9.*6.0-P_wayp.*l6.*l11.*6.0+P_wayp.*l7.*l10.*1.2e+1-P_wayp.*l8.*l9.*6.0+l3.*l6.*l9.*2.0-l5.*l6.*l9.*8.0+l6.*l12.*l15.*9.0-l7.*l11.*l15.*5.0+l8.*l10.*l15+l19.*l32.*l34.*3.0+l28.*l32.*l34.*3.0-l29.*l32.*l34.*3.0+l2.*l12.*t1.*6.0-J_min.*l2.*l11.*t1.*1.8e+1+V_init.*l6.*l11.*t1.*1.8e+1-V_init.*l7.*l10.*t1.*1.8e+1+V_init.*l8.*l9.*t1.*6.0+l2.*l6.*l10.*t1.*1.8e+1-l2.*l7.*l9.*t1.*6.0-A_init.*J_min.*V_init.*l11.*6.0+A_wayp.*J_max.*V_max.*l8.*6.0-A_init.*J_min.*l12.*l14.*2.1e+1+A_init.*V_init.*l6.*l10.*1.2e+1-A_init.*V_init.*l7.*l9.*6.0+A_wayp.*V_max.*l6.*l10.*6.0-A_wayp.*V_max.*l7.*l9.*1.2e+1-J_min.*V_init.*l12.*t1.*6.0+A_init.*l6.*l11.*l14.*2.7e+1+A_init.*l8.*l9.*l14.*3.0-J_min.*J_max.*V_init.*l32.*l34.*6.0+A_init.*l9.*l32.*l34.*t1.*6.0-A_init.*J_min.*J_max.*l32.*l34.*t1.*6.0)./(l8.*l28.*3.0-J_min.*l2.*l11.*3.0-J_min.*l13.*l14.*3.0+V_init.*l6.*l11.*6.0-V_init.*l7.*l10.*1.2e+1+V_init.*l8.*l9.*6.0+l2.*l6.*l10.*6.0-l2.*l7.*l9.*3.0+l6.*l12.*l14.*9.0-l7.*l11.*l14.*9.0-A_init.*J_min.*l12.*t1.*6.0+A_init.*l6.*l11.*t1.*1.8e+1-A_init.*l7.*l10.*t1.*1.8e+1+A_init.*l8.*l9.*t1.*6.0);

l2 = A_wayp.^2;
l3 = J_max.^2;
l4 = t1.^2;
t5 = sqrt(complex(-(J_min-J_max).*(-J_min.*l2+J_max.*l2+A_init.^2.*J_max+J_max.^3.*l4-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_max.*2.0+A_init.*l3.*t1.*2.0-J_min.*l3.*l4-A_init.*J_min.*J_max.*t1.*2.0)))./(J_min.^2-J_min.*J_max);

t7 = (A_wayp-J_min.*t5)./J_max;

t3 = -(A_init+J_max.*t1)./J_min;

t6 = [0.0;0.0];

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

