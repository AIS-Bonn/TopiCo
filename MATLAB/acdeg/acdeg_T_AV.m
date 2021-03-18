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

function [t] = acdeg_T_AV(~,V_init,A_init,~,V_wayp,A_wayp,V_max,~,~,~,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 11:48:12
coder.inline('default');

l2 = A_init.^2;
l3 = A_wayp.^2;
l4 = J_max.^2;
l5 = V_max.^2;
l6 = J_max.*T;
l7 = J_max.*V_init.*2.0;
l8 = J_max.*V_max.*4.0;
l9 = J_max.*V_wayp.*2.0;
l10 = -A_wayp;
l11 = -J_max;
l12 = 1.0./J_min;
l13 = 1.0./J_max;
l14 = -l7;
l16 = -l9;
l19 = J_min+l11;
l20 = J_max.*V_max.*l2.*2.0;
l23 = J_max.*V_max.*l3.*2.0;
l24 = l2.*l3;
l25 = V_init.*V_max.*l4.*4.0;
l26 = V_init.*V_wayp.*l4.*4.0;
l27 = V_max.*V_wayp.*l4.*4.0;
l28 = J_max.*V_wayp.*l2.*-2.0;
l29 = J_max.*V_init.*l3.*-2.0;
l32 = l4.*l5.*4.0;
l30 = -l25;
l31 = -l27;
l33 = l20+l23+l24+l26+l28+l29+l30+l31+l32;
l34 = sqrt(complex(l33));
l35 = l34.*2.0;
l36 = l2+l3+l8+l14+l16+l35;
l41 = l12.*l19.*(l2+l3+l8+l14+l16-l35);
l38 = l12.*l19.*l36;
l42 = sqrt(complex(l41));
l39 = sqrt(complex(l38));
t4 = [l13.*(A_init+l6+l10-l39);l13.*(A_init+l6+l10+l42);l13.*(A_init+l6+l10+l39);l13.*(A_init+l6+l10-l42)];

l2 = J_max.^2;
t1 = -(J_min.*V_init.*-2.0+J_max.*V_init.*2.0+J_min.*V_wayp.*2.0-J_max.*V_wayp.*2.0+A_init.^2+A_wayp.^2-A_init.*A_wayp.*2.0+A_init.*J_max.*T.*2.0-A_wayp.*J_min.*T.*2.0-A_init.*J_max.*t4.*2.0+A_wayp.*J_min.*t4.*2.0+J_min.*J_max.*T.^2+J_min.*J_max.*t4.^2-J_min.*J_max.*T.*t4.*2.0)./(T.*l2.*2.0-l2.*t4.*2.0-A_init.*J_min.*2.0+A_init.*J_max.*2.0+A_wayp.*J_min.*2.0-A_wayp.*J_max.*2.0-J_min.*J_max.*T.*2.0+J_min.*J_max.*t4.*2.0);

t7 = (A_init-A_wayp-J_min.*t1+J_max.*t1-J_min.*t4+J_min.*T)./(J_min-J_max);

t5 = (A_wayp-J_max.*t7)./J_min;

t3 = -(A_init+J_max.*t1)./J_min;

t6 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


