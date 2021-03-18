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

function [t] = aceg_T_V(~,V_init,A_init,~,V_wayp,~,~,V_min,~,~,J_max,J_min,T) %#codegen
% Generated on 02-Sep-2019 15:22:52
coder.inline('default');

l2 = J_max.*T;
l3 = -J_max;
l4 = -V_min;
l5 = sqrt(2.0);
l6 = sqrt(J_max);
l7 = J_min+l3;
l8 = V_wayp+l4;
l9 = 1.0./l7;
l10 = sqrt(complex(l8));
l11 = l5.*l6.*l10;
t3 = [-l9.*(A_init+l2+l11);-l9.*(A_init+l2-l11)];

l2 = -t3;
l3 = T+l2;
t1 = -(V_init.*2.0-V_wayp.*2.0+A_init.*t3.*2.0+J_max.*l3.^2+J_min.*t3.^2+l3.*(A_init+J_min.*t3).*2.0)./(J_max.*l3.*2.0-J_min.*t3.*2.0+J_max.*t3.*2.0-J_max.*(T.*2.0-t3.*2.0));

t7 = T-t1-t3;

t2 = [0.0;0.0];

t6 = [0.0;0.0];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
