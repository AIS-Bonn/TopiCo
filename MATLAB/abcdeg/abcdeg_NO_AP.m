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

function [t] = abcdeg_NO_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 04-Sep-2019 17:17:26
coder.inline('default');

t7 = [0.0;(A_wayp.*2.0)./J_max];

l2 = J_max.*t7;
l3 = A_init.^2;
l4 = A_min.^2;
l5 = A_min.^3;
l7 = A_wayp.^2;
l8 = J_min.^2;
l9 = J_max.^2;
l15 = J_min.*J_max.*V_init.*2.0;
l16 = J_min.*J_max.*V_max.*2.0;
l6 = l4.^2;
l14 = -l2;
l17 = J_max.*l3;
l18 = J_min.*l4;
l19 = J_max.*l4;
l20 = -l15;
l21 = A_wayp.*J_min.*l2.*2.0;
l25 = A_wayp.*J_max.*l2.*2.0;
l26 = J_max.*l2.^2;
l27 = J_min.*l2.^2;
l22 = -l19;
l23 = -l21;
l24 = A_wayp+l14;
l28 = J_max.*l2.*l14;
l29 = l16+l17+l18+l20+l22+l23+l25+l27+l28;
t4 = -(l6.*l8.*8.0-l6.*l9.*2.0+l17.*l18.*1.2e+1+l18.*l26.*6.0-l17.*l29.*6.0-l18.*l29.*1.2e+1+l29.^2.*3.0-l2.^2.*l4.*l9.*6.0-J_min.*J_max.*l6.*6.0-A_wayp.*l5.*l9.*1.2e+1-V_init.*l8.*l19.*1.2e+1+V_init.*l8.*l26.*1.2e+1-V_init.*l9.*l27.*1.2e+1+V_max.*l9.*l18.*1.2e+1+A_init.^3.*A_min.*l9.*8.0+l2.*l5.*l9.*1.2e+1-l3.*l4.*l9.*6.0+l5.*l9.*l24.*1.2e+1+A_min.*l2.^3.*l8.*1.6e+1+A_min.*l2.^3.*l9.*1.2e+1+A_min.*l9.*l24.^3.*4.0-V_init.^2.*l8.*l9.*2.4e+1-A_min.*A_wayp.*l2.^2.*l8.*2.4e+1-A_min.*A_wayp.*l2.^2.*l9.*3.6e+1+A_min.*l2.^2.*l8.*l24.*1.2e+1+A_min.*A_wayp.*J_min.*l26.*6.0e+1-A_min.*J_min.*l2.*l26.*2.4e+1-A_wayp.*J_max.*l2.*l18.*1.2e+1+A_min.*P_init.*l8.*l9.*2.4e+1-A_min.*P_wayp.*l8.*l9.*2.4e+1+J_min.*V_init.*l3.*l9.*1.2e+1+V_init.*V_max.*l8.*l9.*2.4e+1+A_min.*l2.*l7.*l9.*2.4e+1+A_wayp.*l2.*l4.*l9.*1.2e+1-A_min.*J_min.*J_max.*l2.*l7.*2.4e+1+A_min.*J_min.*V_init.*l2.*l9.*2.4e+1+A_min.*J_min.*V_init.*l9.*l24.*2.4e+1-A_min.*J_min.*V_max.*l2.*l9.*2.4e+1+A_min.*J_max.*V_max.*l2.*l8.*2.4e+1+A_wayp.*J_min.*V_init.*l2.*l9.*2.4e+1-A_wayp.*J_max.*V_init.*l2.*l8.*2.4e+1+A_min.*J_min.*J_max.*l2.*l24.^2.*1.2e+1-A_init.*A_min.*J_min.*V_init.*l9.*2.4e+1-A_min.*A_wayp.*J_min.*V_init.*l9.*2.4e+1+A_min.*A_wayp.*J_min.*V_max.*l9.*2.4e+1)./(A_min.*l8.*l26.*1.2e+1-A_min.*l9.*l27.*1.2e+1+A_min.*V_max.*l8.*l9.*2.4e+1+A_min.*A_wayp.*J_min.*l2.*l9.*2.4e+1-A_min.*A_wayp.*J_max.*l2.*l8.*2.4e+1);

l2 = J_max.*t7;
l3 = -A_min;
l4 = -l2;
l5 = A_init+l3;
l6 = A_wayp+l4;
t2 = ((J_min.*V_init.*2.0-J_min.*V_max.*2.0-A_init.*l5.*2.0-A_wayp.^2+l5.^2+l6.^2+J_min.*l2.*t7+J_min.*l6.*t7.*2.0+(A_min.*J_min.*l3)./J_max).*(-1.0./2.0))./(A_min.*J_min);

t5 = (A_wayp-J_max.*t7)./J_min;

l2 = 1.0./J_max;
l3 = A_min.*l2;
l4 = -l3;
t3 = [l4;l4];

l2 = -A_min;
l3 = 1.0./J_min;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t6 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

