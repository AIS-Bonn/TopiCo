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

function [t] = aceg_T_A(~,V_init,A_init,~,~,A_wayp,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 12:37:25
coder.inline('default');

l2 = J_min.*T;
l3 = J_max.*T;
l4 = -A_wayp;
l5 = -J_max;
l6 = J_min+l5;
l8 = A_init+l2+l4;
l9 = A_init+l3+l4;
l7 = l6.^2;
t7 = (J_min.^2.*l9.^2+J_min.*V_init.*l7.*2.0-J_min.*V_max.*l7.*2.0+A_wayp.*l4.*l7-J_min.*l9.*(J_max.*l2+J_max.*l4+A_init.*J_min).*2.0+J_min.*J_max.*l8.^2+A_init.*J_min.*l6.*l8.*2.0)./(J_min.*l9.*(J_min.*l5+J_max.^2).*2.0+A_init.*J_min.*l7.*2.0-A_wayp.*J_min.*l7.*2.0+J_min.*J_max.*l6.*l8.*2.0);

t3 = -(A_init-A_wayp+J_max.*T)./(J_min-J_max);

t1 = -(A_init-A_wayp+J_min.*t3+J_max.*t7)./J_max;

t6 = 0.0;

t5 = 0.0;

t4 = 0.0;

t2 = 0.0;

t = [t1, t2, t3, t4, t5, t6, t7];

end
