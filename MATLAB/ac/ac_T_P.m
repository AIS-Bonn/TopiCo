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

function [t] = ac_T_P(P_init,V_init,A_init,P_wayp,~,~,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 14:04:13
coder.inline('default');

l2 = P_init.*6.0;
l3 = P_wayp.*6.0;
l4 = T.^2;
l5 = T.^3;
l6 = T.*V_init.*6.0;
l7 = -J_max;
l9 = J_min.*T.*3.0;
l10 = J_max.*T.*3.0;
l12 = sqrt(3.0);
l8 = -l3;
l11 = J_min.*l5;
l13 = -l10;
l14 = J_min+l7;
l15 = A_init.*l4.*3.0;
l16 = J_min.*l4.*3.0;
l17 = J_max.*l4.*3.0;
l18 = -l17;
l19 = 1.0./l14;
l22 = l9+l13;
l26 = l2+l6+l8+l11+l15;
l20 = l19.^2;
l21 = l19.^3;
l23 = l22.^3;
l24 = l16+l18;
l25 = (l19.*l22)./3.0;
l28 = l19.*l26;
l27 = l21.*l23.*(2.0./2.7e+1);
l29 = (l20.*l22.*l24)./3.0;
l30 = -l29;
l31 = l27+l28+l30;
l32 = l31.^(1.0./3.0);
l33 = l32./2.0;
l35 = l12.*l32.*5.0e-1i;
l34 = -l33;
t1 = [l25+l32;l25+l34-l35;l25+l34+l35];

t3 = T-t1;

t2 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t7 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
