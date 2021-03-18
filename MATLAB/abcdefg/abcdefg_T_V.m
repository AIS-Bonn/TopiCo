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

function [t] = abcdefg_T_V(~,V_init,A_init,~,V_wayp,~,V_max,V_min,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 02-Sep-2019 15:54:56
coder.inline('default');

l2 = 1.0./J_max;
l3 = -V_min;
l4 = sqrt(2.0);
l5 = sqrt(J_max);
l6 = V_wayp+l3;
l7 = sqrt(l6);
l8 = l4.*l5.*l7;
t7 = [-l2.*(A_min+l8);-l2.*(A_min-l8)];

l2 = A_init.^2;
l3 = A_min.^2;
l4 = A_max.^2;
t4 = (J_min.*l2+J_max.*l4-(A_max.^3.*J_max+A_min.*J_min.*l2+A_min.*J_min.*l4+A_max.*J_min.*l2+A_max.*J_min.*l3+A_min.*J_max.*l4+A_max.*J_max.*l3-A_min.*J_min.*J_max.*V_init.*2.0+A_min.*J_min.*J_max.*V_max.*2.0+A_max.*J_min.*J_max.*V_min.*2.0+A_min.*A_max.*J_min.*J_max.*t7.*2.0)./A_max+A_init.*A_min.*J_min.*2.0+A_min.*A_max.*J_max.*2.0+J_min.*J_max.*V_max.*2.0+A_min.*J_min.*J_max.*T.*2.0)./(A_min.*J_min.*J_max.*2.0);

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

l2 = A_min.^2;
l3 = A_max.^2;
t6 = (J_min.*l2-J_min.*l3-J_max.*l2+J_max.*l3+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_max.*J_min.*J_max.*t2.*2.0)./(A_min.*J_min.*J_max.*2.0);

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
