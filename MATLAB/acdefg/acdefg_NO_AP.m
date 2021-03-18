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

function [t] = acdefg_NO_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = J_min.^2;
l4 = A_init.*J_min;
l5 = A_init.*J_max;
l6 = J_min.*J_max;
l7 = J_min.*V_init.*2.0;
l8 = J_min.*V_max.*2.0;
l9 = -J_max;
l11 = sqrt(J_max);
l10 = -l7;
l12 = -l6;
l13 = J_min+l9;
l14 = l3+l12;
l16 = l2+l8+l10;
l15 = 1.0./l14;
l17 = l13.*l16;
l18 = -l17;
l19 = sqrt(complex(l18));
l20 = l11.*l19;
t1 = [l15.*(-l4+l5+l20);-l15.*(l4-l5+l20)];

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l6 = A_wayp.^2;
l7 = A_wayp.^3;
l9 = J_min.^2;
l10 = J_min.^3;
l12 = J_max.^2;
l13 = J_min.^5;
l14 = t1.^2;
l15 = t1.^3;
l5 = l4.^2;
l8 = l6.^2;
l11 = l9.^2;
l16 = l14.^2;
t4 = ((l5.*l9-l5.*l12-l8.*l9.*3.0-l8.*l12.*3.0+l2.^2.*l9.*3.0+l9.^3.*l16.*3.0+J_min.*J_max.*l8.*6.0+A_init.*l13.*l15.*1.2e+1-A_min.*l3.*l9.*8.0+A_min.*l7.*l9.*8.0-A_min.*l13.*l15.*8.0-J_max.*l13.*l16.*6.0-l4.*l6.*l9.*6.0+l2.*l11.*l14.*1.8e+1+l11.*l12.*l16.*3.0+l3.*l10.*t1.*1.2e+1+V_init.^2.*l9.*l12.*1.2e+1-V_max.^2.*l9.*l12.*1.2e+1-J_max.*l2.*l10.*l14.*3.0e+1+J_max.*l4.*l10.*l14.*6.0-J_max.*l3.*l9.*t1.*1.2e+1+V_init.*l10.*l12.*l14.*1.2e+1+l2.*l9.*l12.*l14.*1.2e+1-l4.*l9.*l12.*l14.*6.0-A_min.*J_min.*J_max.*l7.*1.2e+1-A_init.*A_min.*l11.*l14.*2.4e+1-A_init.*J_max.*l11.*l15.*2.4e+1+A_min.*J_max.*l11.*l15.*1.2e+1-A_min.*P_init.*l9.*l12.*2.4e+1+A_min.*P_wayp.*l9.*l12.*2.4e+1+J_min.*J_max.*l2.*l4.*6.0+J_min.*J_max.*l4.*l6.*6.0-J_max.*V_init.*l2.*l9.*1.2e+1-J_min.*V_init.*l4.*l12.*1.2e+1-J_max.*V_init.*l11.*l14.*1.2e+1+J_max.*V_max.*l4.*l9.*1.2e+1+J_max.*V_max.*l6.*l9.*1.2e+1-J_min.*V_max.*l6.*l12.*1.2e+1+A_init.*l10.*l12.*l15.*1.2e+1-A_min.*l10.*l12.*l15.*4.0-A_min.*l2.*l10.*t1.*2.4e+1+A_init.*A_min.*J_max.*l10.*l14.*3.6e+1-A_init.*J_max.*V_init.*l10.*t1.*2.4e+1+A_min.*J_max.*V_init.*l10.*t1.*2.4e+1-A_init.*A_min.*l9.*l12.*l14.*1.2e+1+A_init.*J_max.*l4.*l9.*t1.*1.2e+1-A_init.*J_min.*l4.*l12.*t1.*1.2e+1+A_min.*J_max.*l2.*l9.*t1.*2.4e+1+A_init.*V_init.*l9.*l12.*t1.*2.4e+1-A_min.*V_init.*l9.*l12.*t1.*2.4e+1+A_init.*A_min.*J_max.*V_init.*l9.*2.4e+1-A_min.*A_wayp.*J_max.*V_max.*l9.*2.4e+1).*(-1.0./1.2e+1))./(A_min.*J_max.*l2.*l9+A_min.*J_max.*l11.*l14-A_min.*V_init.*l9.*l12.*2.0-A_min.*l10.*l12.*l14+A_init.*A_min.*J_max.*l10.*t1.*2.0-A_init.*A_min.*l9.*l12.*t1.*2.0);

l2 = -A_wayp;
l3 = 1.0./J_min;
l4 = 1.0./J_max;
l5 = A_min+l2;
t6 = (l3.*l4.*(A_wayp.^2.*J_max+J_min.*(A_init+J_min.*t1).^2-J_min.*J_max.*(V_init.*2.0+A_init.*t1.*2.0+A_min.^2.*l3+J_min.*t1.^2+l4.*l5.^2-A_min.*l4.*l5.*2.0)+J_min.*J_max.*V_max.*2.0))./(A_min.*2.0);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

t3 = -(A_init+J_min.*t1)./J_max;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
