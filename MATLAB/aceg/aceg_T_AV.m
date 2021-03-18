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

function [t] = aceg_T_AV(~,V_init,A_init,~,V_wayp,A_wayp,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 11:12:37
coder.inline('default');

l2 = J_max.*T;
l3 = -A_wayp;
l4 = -J_max;
l5 = J_min+l4;
l7 = A_init+l2+l3;
l6 = l5.^2;
l8 = l7.^2;
t7 = (A_init.^2.*l6-A_wayp.^2.*l6-J_min.^2.*l8+J_min.*J_max.*l8-J_max.*V_init.*l6.*2.0+J_max.*V_wayp.*l6.*2.0-A_wayp.*J_min.*l5.*l7.*2.0+A_wayp.*J_max.*l5.*l7.*2.0)./(J_max.^2.*l5.*l7.*2.0-J_min.*J_max.*l5.*l7.*2.0);

t3 = -(A_init-A_wayp+J_max.*T)./(J_min-J_max);

t1 = -(A_init-A_wayp+J_min.*t3+J_max.*t7)./J_max;

t6 = 0.0;

t5 = 0.0;

t4 = 0.0;

t2 = 0.0;

t = [t1, t2, t3, t4, t5, t6, t7];

end
