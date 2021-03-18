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

function [t] = ac_T_V(~,V_init,A_init,~,V_wayp,~,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 16:29:55
coder.inline('default');

l2 = T.^2;
l3 = V_init.*2.0;
l4 = V_wayp.*2.0;
l5 = J_min.*T;
l6 = J_max.*T;
l7 = -J_max;
l9 = A_init.*T.*2.0;
l8 = -l4;
l10 = J_max.*l2;
l11 = J_min+l7;
l12 = 1.0./l11;
l13 = l3+l8+l9+l10;
l14 = l11.*l13;
l15 = -l14;
l16 = sqrt(complex(l15));
t1 = [l12.*(l5-l6+l16);-l12.*(-l5+l6+l16)];

t3 = T-t1;

t2 = [0.0;0.0];

t4 = [0.0;0.0];

t5 = [0.0;0.0];

t6 = [0.0;0.0];

t7 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
