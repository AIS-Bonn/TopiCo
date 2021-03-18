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

function [t] = ac_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,~,~,~,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_wayp.^2;
l5 = A_wayp.^3;
l6 = J_min.^2;
l7 = J_min.^3;
l8 = J_max.^2;
l10 = A_wayp.*J_min.*J_max.*6.0;
l11 = A_init.*J_max.*V_init.*6.0;
l12 = A_wayp.*J_max.*V_init.*6.0;
l13 = J_min.*J_max.*V_init.*6.0;
l14 = sqrt(3.0);
l9 = l3.*2.0;
l15 = -l10;
l16 = -l11;
l17 = A_wayp.*l2.*3.0;
l19 = J_min.*l2.*3.0;
l20 = J_max.*l2.*3.0;
l21 = A_wayp.*l6.*3.0;
l22 = J_min.*l4.*3.0;
l23 = A_wayp.*l8.*3.0;
l24 = J_max.*l4.*3.0;
l25 = J_min.*l8.*2.0;
l26 = J_max.*l6.*3.0;
l27 = P_init.*l8.*6.0;
l28 = P_wayp.*l8.*6.0;
l29 = V_init.*l8.*6.0;
l30 = -l17;
l33 = -l26;
l34 = -l28;
l36 = l15+l21+l23;
l35 = l7+l25+l33;
l40 = l36.^2;
l41 = l36.^3;
l43 = l5+l9+l12+l16+l27+l30+l34;
l37 = 1.0./l35;
l38 = l37.^2;
l39 = l37.^3;
l44 = (l36.*l37)./3.0;
l47 = l37.*(l13-l19+l20+l22-l24-l29).*(-1.0./3.0);
l48 = (l37.*l43)./2.0;
l45 = (l38.*l40)./9.0;
l46 = (l39.*l41)./2.7e+1;
l49 = l36.*l38.*(l13-l19+l20+l22-l24-l29).*(-1.0./6.0);
l50 = l45+l47;
l53 = l46+l48+l49;
l51 = l50.^3;
l54 = l53.^2;
l52 = -l51;
l55 = l52+l54;
l56 = sqrt(complex(l55));
l57 = l53+l56;
l58 = l57.^(1.0./3.0);
l59 = 1.0./l58;
l60 = l58./2.0;
l61 = -l60;
l62 = l50.*l59;
l63 = -l62;
l64 = l62./2.0;
l65 = -l64;
l66 = l58+l63;
l67 = l14.*l66.*5.0e-1i;
t3 = [l44+l58+l62;l44+l61+l65-l67;l44+l61+l65+l67];

t1 = -(A_init-A_wayp+J_min.*t3)./J_max;

t7 = [0.0;0.0;0.0];

t6 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t2 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


