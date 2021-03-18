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

function [t] = abc_T_V(~,V_init,A_init,~,V_wayp,~,~,~,A_max,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 16:31:13
coder.inline('default');

l2 = A_init.^2;
l3 = A_max.^2;
l4 = A_init.*J_min;
l5 = A_max.*J_min;
l6 = J_max.*V_init.*2.0;
l7 = J_max.*V_wayp.*2.0;
l8 = J_min.*J_max.*T;
l9 = 1.0./J_min;
l10 = 1.0./J_max;
l11 = A_init.*A_max.*2.0;
l13 = A_max.*J_max.*T.*2.0;
l14 = sqrt(J_max);
l19 = J_min.*(l2+l3-l6+l7-l11-l13);
l20 = sqrt(complex(l19));
l21 = l14.*l20;
t2 = [l9.*l10.*(l4-l5+l8+l21);l9.*l10.*(l4-l5+l8-l21)];

t3 = T-t2+(A_init-A_max)./J_max;

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t4 = [0.0;0.0];

t5 = [0.0;0.0];

t6 = [0.0;0.0];

t7 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


