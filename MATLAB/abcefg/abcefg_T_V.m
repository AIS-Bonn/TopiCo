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

function [t] = abcefg_T_V(~,V_init,A_init,~,V_wayp,~,~,V_min,A_max,A_min,J_max,J_min,T) %#codegen
% Generated on 02-Sep-2019 15:45:15
coder.inline('default');

l2 = A_init.^2;
l3 = A_min.^2;
l4 = A_max.^2;
l5 = A_min.*J_min.*J_max;
l6 = A_max.*J_min.*J_max;
l7 = -V_min;
l8 = A_init.*A_max.*J_min.*2.0;
l9 = A_min.*A_max.*J_min.*2.0;
l10 = A_min.*A_max.*J_max.*2.0;
l11 = J_min.*J_max.*V_init.*2.0;
l12 = J_min.*J_max.*V_min.*2.0;
l18 = sqrt(2.0);
l19 = sqrt(J_max);
l13 = J_min.*l2;
l14 = J_min.*l3;
l15 = J_max.*l3;
l16 = J_min.*l4;
l17 = J_max.*l4;
l20 = -l6;
l21 = V_wayp+l7;
l22 = T.*l6.*2.0;
l23 = sqrt(complex(l21));
l24 = l5+l20;
l25 = 1.0./l24;
l26 = A_max.*J_min.*l18.*l19.*l23.*2.0;
t6 = [l25.*(l8+l9-l10+l11-l12-l13-l14+l15-l16+l17+l22-l26).*(-1.0./2.0);l25.*(l8+l9-l10+l11-l12-l13-l14+l15-l16+l17+l22+l26).*(-1.0./2.0)];

l2 = A_min.^2;
l3 = A_max.^2;
t7 = ((J_min.*l2+J_min.*l3-J_max.*l2-J_max.*l3+A_init.^2.*J_min-A_init.*A_max.*J_min.*2.0+A_min.*A_max.*J_max.*2.0-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_max.*J_min.*J_max.*T.*2.0-A_min.*J_min.*J_max.*t6.*2.0+A_max.*J_min.*J_max.*t6.*2.0).*(-1.0./2.0))./(A_max.*J_min.*J_max);

l2 = A_min.^2;
l3 = A_max.^2;
t2 = (J_min.*l2-J_min.*l3-J_max.*l2+J_max.*l3+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_min.*2.0-A_min.*J_min.*J_max.*t6.*2.0)./(A_max.*J_min.*J_max.*2.0);

l2 = -A_max;
l3 = 1.0./J_min;
l4 = A_min+l2;
l5 = l3.*l4;
t3 = [l5;l5];

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6];

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
