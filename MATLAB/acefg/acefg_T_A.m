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

function [t] = acefg_T_A(~,V_init,A_init,~,~,A_wayp,V_max,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 12:41:01
coder.inline('default');

l2 = A_init.^2;
l3 = A_wayp.^2;
l4 = J_min.^2;
l5 = J_min.*J_max;
l6 = -J_max;
l7 = A_init.*A_min.*J_min.*2.0;
l8 = A_min.*A_wayp.*J_min.*2.0;
l9 = V_init.*l5.*2.0;
l10 = V_max.*l5.*2.0;
l11 = J_min.*l2;
l12 = J_min.*l3;
l13 = J_max.*l3;
l14 = -l5;
l17 = J_min+l6;
l18 = A_min.*T.*l5.*2.0;
l21 = l4+l14;
l24 = -l17.*(l7-l8+l9-l10-l11+l12-l13+l18);
l22 = 1.0./l21;
l25 = sqrt(complex(l24));
l26 = l22.*l25;
t3 = [l26;-l26];

t6 = (A_init-A_wayp+J_min.*t3-J_max.*t3+J_max.*T)./J_max;

t1 = -(A_init-A_min+J_min.*t3)./J_max;

t7 = -(A_init-A_wayp+J_min.*t3+J_max.*t1)./J_max;

t5 = [0.0;0.0];

t4 = [0.0;0.0];

t2 = [0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


