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

function [t] = acdefg_T_V(~,V_init,A_init,~,V_wayp,~,V_max,V_min,~,A_min,J_max,J_min,T) %#codegen
% Generated on 02-Sep-2019 16:18:40
coder.inline('default');

l2 = 1.0./J_max;
l3 = -V_min;
l4 = sqrt(2.0);
l5 = sqrt(J_max);
l6 = V_wayp+l3;
l7 = sqrt(complex(l6));
l8 = l4.*l5.*l7;
t7 = [-l2.*(A_min+l8);-l2.*(A_min-l8)];

l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_min.*J_max;
l5 = J_max.*V_init.*2.0;
l6 = J_max.*V_max.*2.0;
l7 = -J_max;
l8 = -l5;
l9 = -l4;
l10 = J_min+l7;
l11 = l3+l9;
l13 = l2+l6+l8;
l12 = 1.0./l11;
l14 = J_min.*l10.*l13;
l15 = sqrt(complex(l14));
l16 = l12.*l15;
t3 = [l16;l16];

l2 = A_min.^2;
l3 = t3.^2;
t6 = (l2-J_max.*V_init.*2.0+J_max.*V_min.*2.0-J_min.^2.*l3+A_init.^2+J_min.*J_max.*l3-(J_max.*l2)./J_min)./(A_min.*J_max.*2.0);

t4 = (A_init+J_min.*t3-J_max.*(t3+t6+t7+A_min./J_min)+J_max.*T)./J_max;

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

t1 = -(A_init+J_min.*t3)./J_max;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
