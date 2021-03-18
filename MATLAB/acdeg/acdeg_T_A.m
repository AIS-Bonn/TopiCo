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

function [t] = acdeg_T_A(~,V_init,A_init,~,~,A_wayp,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 16:29:55
coder.inline('default');

t7 = [0.0;(A_wayp.*2.0)./J_max];

l2 = A_init.^2;
l3 = J_max.^2;
l4 = J_max.^3;
l5 = t7.^2;
l6 = -J_min;
l7 = -J_max;
l8 = 1.0./J_min;
l9 = J_min.*J_max.*V_init.*2.0;
l10 = J_min.*J_max.*V_max.*2.0;
l15 = A_wayp.*J_min.*J_max.*t7.*2.0;
l12 = -l10;
l13 = J_min+l7;
l14 = J_max+l6;
l16 = l2.*l6;
l17 = A_wayp.*l3.*t7.*2.0;
l18 = l4.*l5;
l21 = l3.*l5.*l6;
l20 = -l17;
l22 = l9+l12+l15+l16+l18+l20+l21;
l23 = sqrt(complex(l22));
t4 = (A_init.*l13+sqrt(complex(l14)).*l23+J_max.*T.*l13+l7.*l13.*(t7+l8.*(A_wayp+l7.*t7)-l8.*1.0./sqrt(complex(l14)).*l23))./(J_max.*l13);

l2 = J_max.^2;
l3 = t7.^2;
t3 = sqrt(complex((J_min-J_max).*(A_init.^2.*J_min-J_max.^3.*l3-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_max.*2.0+A_wayp.*l2.*t7.*2.0+J_min.*l2.*l3-A_wayp.*J_min.*J_max.*t7.*2.0)))./(J_min.^2-J_min.*J_max);

t5 = (A_wayp-J_max.*t7)./J_min;

t1 = -(A_init+J_min.*t3)./J_max;

t6 = [0.0;0.0];

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

