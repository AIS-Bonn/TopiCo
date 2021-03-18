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

function [t] = abceg_T_V(~,V_init,A_init,~,V_wayp,~,~,V_min,A_max,~,J_max,J_min,T) %#codegen
% Generated on 02-Sep-2019 15:40:56
coder.inline('default');

l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_min.*J_max;
l5 = J_max.*V_init.*2.0;
l6 = J_max.*V_min.*2.0;
l7 = -J_max;
l8 = -V_wayp;
l9 = A_init.*A_max.*2.0;
l11 = A_max.*J_max.*T.*2.0;
l12 = sqrt(2.0);
l13 = sqrt(J_max);
l10 = -l6;
l14 = -l4;
l15 = -l2;
l16 = J_min+l7;
l17 = V_min+l8;
l18 = l3+l14;
l19 = sqrt(complex(l17));
l20 = 1.0./l18;
l21 = A_max.*l12.*l13.*l19.*2.0i;
l23 = l5+l9+l10+l11+l15+l21;
l26 = -J_min.*l16.*(l2-l5+l6-l9-l11+l21);
l25 = J_min.*l16.*l23;
l28 = sqrt(complex(l26));
l27 = sqrt(complex(l25));
l30 = l20.*l28;
l29 = l20.*l27;
t3 = [l30;l29;-l30;-l29];

l2 = J_min.*t3;
l3 = A_max+l2;
t7 = -(J_max.*V_init.*2.0-J_max.*V_min.*2.0+A_init.^2-A_max.^2-l3.^2-A_init.*(A_init-A_max).*2.0+A_max.*J_max.*T.*2.0+J_max.*l2.*t3)./(J_max.*l2.*2.0-J_max.*l3.*2.0);

l2 = J_min.*t3;
l3 = -A_max;
l4 = A_init+l3;
t2 = ((J_max.*V_init.*2.0-J_max.*V_min.*2.0-A_init.*l4.*2.0-(A_max+l2+J_max.*t7).^2+l4.^2+J_max.^2.*t7.^2+J_max.*t7.*(A_max+l2).*2.0+A_max.*J_max.*t3.*2.0+J_max.*l2.*t3).*(-1.0./2.0))./(A_max.*J_max);

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


