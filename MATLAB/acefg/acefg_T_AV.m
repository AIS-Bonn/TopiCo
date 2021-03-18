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

function [t] = acefg_T_AV(~,V_init,A_init,~,V_wayp,A_wayp,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 11:28:47
coder.inline('default');

l2 = A_init.^2;
l3 = A_wayp.^2;
l4 = J_max.^2;
l5 = A_init.*J_min;
l6 = A_init.*J_max;
l7 = A_min.*J_min;
l8 = A_min.*J_max;
l9 = J_min.*J_max;
l10 = J_max.*V_init.*2.0;
l11 = J_max.*V_wayp.*2.0;
l12 = -J_max;
l13 = A_init.*A_min.*2.0;
l14 = A_min.*A_wayp.*2.0;
l15 = -l11;
l16 = T.*l8.*2.0;
l17 = -l14;
l18 = -l9;
l19 = -l2;
l20 = J_min+l12;
l21 = l4+l18;
l23 = l3+l10+l13+l15+l16+l17+l19;
l22 = 1.0./l21;
l24 = J_min.*l20.*l23;
l25 = -l24;
l26 = sqrt(complex(l25));
t1 = [l22.*(l5-l6-l7+l8+l26);-l22.*(-l5+l6+l7-l8+l26)];

t6 = (A_init-A_min-J_min.*t1+J_max.*t1+J_min.*T+(J_min.*(A_min-A_wayp))./J_max)./J_min;

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

t3 = -(A_init-A_min+J_max.*t1)./J_min;

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
