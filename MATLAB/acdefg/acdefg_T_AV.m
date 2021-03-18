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

function [t] = acdefg_T_AV(~,V_init,A_init,~,V_wayp,A_wayp,V_max,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 11:41:15
coder.inline('default');

l2 = A_init.^2;
l3 = J_max.^2;
l4 = A_init.*J_min;
l5 = A_init.*J_max;
l6 = J_min.*J_max;
l7 = J_max.*V_init.*2.0;
l8 = J_max.*V_max.*2.0;
l9 = -J_max;
l10 = -l7;
l11 = -l6;
l12 = J_min+l9;
l13 = l3+l11;
l15 = l2+l8+l10;
l14 = 1.0./l13;
l16 = J_min.*l12.*l15;
l17 = sqrt(complex(l16));
t1 = [l14.*(l4-l5+l17);-l14.*(-l4+l5+l17)];

l2 = A_min.^2;
l3 = J_max.^2;
l4 = t1.^2;
t4 = (J_min.*l2-J_max.*l2-A_init.^2.*J_max+A_wayp.^2.*J_min-J_max.^3.*l4+A_init.*A_min.*J_max.*2.0-A_min.*A_wayp.*J_min.*2.0+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_wayp.*2.0-A_init.*l3.*t1.*2.0+A_min.*l3.*t1.*2.0+J_min.*l3.*l4+A_min.*J_min.*J_max.*T.*2.0+A_init.*J_min.*J_max.*t1.*2.0-A_min.*J_min.*J_max.*t1.*2.0)./(A_min.*J_min.*J_max.*2.0);

l2 = -A_wayp;
l3 = A_min+l2;
t6 = ((A_min.^2.*J_max+J_min.*l3.^2-J_max.*(A_init+J_max.*t1).^2+J_min.*J_max.^2.*t1.^2+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_wayp.*2.0-A_min.*J_min.*l3.*2.0+A_init.*J_min.*J_max.*t1.*2.0).*(-1.0./2.0))./(A_min.*J_min.*J_max);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

l2 = 1.0./J_min;
l3 = A_min.*l2;
t5 = [l3;l3];

t3 = -(A_init+J_max.*t1)./J_min;

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
