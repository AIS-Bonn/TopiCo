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

function [t] = abcefg_O_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,V_max,~,A_max,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_min.^3;
l8 = A_max.^2;
l9 = A_max.^3;
l11 = A_wayp.^2;
l12 = A_wayp.^3;
l14 = J_min.^2;
l15 = J_max.^2;
l16 = V_init.^2;
l17 = V_max.^2;
l18 = -A_max;
l19 = sqrt(3.0);
l20 = A_min.*A_max.*J_min.*J_max;
l24 = A_min.*J_min.*J_max.*V_init.*6.0;
l25 = A_max.*J_min.*J_max.*V_init.*6.0;
l4 = l2.^2;
l7 = l5.^2;
l10 = l8.^2;
l13 = l11.^2;
l21 = A_min+l18;
l22 = J_min.*l9.*3.0;
l23 = J_max.*l9.*3.0;
l26 = J_min.*J_max.*l8;
l27 = A_min.*J_min.*J_max.*l18;
l28 = A_min.*J_min.*l2.*3.0;
l29 = A_max.*J_min.*l2.*3.0;
l30 = A_min.*J_min.*l8.*3.0;
l31 = A_max.*J_max.*l5.*3.0;
l32 = A_min.*J_max.*l8.*6.0;
l42 = l12.*l20.*1.2e+1;
l46 = A_min.*A_max.*l3.*l14.*8.0;
l47 = A_min.*A_max.*l12.*l14.*8.0;
l48 = l5.*l9.*l15.*3.0;
l49 = l6.*l8.*l15.*3.0;
l50 = A_init.*A_min.*A_max.*J_max.*V_init.*l14.*2.4e+1;
l51 = A_min.*A_max.*A_wayp.*J_max.*V_max.*l14.*2.4e+1;
l52 = A_max.*J_min.*J_max.*l5.*l11.*6.0;
l57 = A_min.*A_max.*P_init.*l14.*l15.*2.4e+1;
l58 = A_min.*A_max.*P_wayp.*l14.*l15.*2.4e+1;
l59 = A_min.*J_max.*V_init.*l2.*l14.*1.2e+1;
l60 = A_min.*J_max.*V_init.*l8.*l14.*1.2e+1;
l61 = A_max.*J_max.*V_max.*l5.*l14.*1.2e+1;
l62 = A_max.*J_min.*V_max.*l11.*l15.*1.2e+1;
l63 = A_max.*J_max.*V_max.*l11.*l14.*1.2e+1;
l64 = A_min.*l2.*l8.*l14.*6.0;
l65 = A_max.*l5.*l11.*l14.*6.0;
l70 = A_min.*l14.*l15.*l16.*1.2e+1;
l71 = A_max.*l14.*l15.*l17.*1.2e+1;
l33 = A_max.*J_min.*J_max.*l13.*6.0;
l34 = A_min.*l10.*l14;
l36 = A_min.*l10.*l15;
l37 = A_max.*l7.*l15;
l39 = A_min.*l4.*l14.*3.0;
l40 = A_max.*l13.*l14.*3.0;
l41 = A_max.*l13.*l15.*3.0;
l44 = l7.*l14.*l18;
l53 = -l47;
l54 = -l49;
l55 = -l50;
l56 = -l52;
l66 = -l58;
l67 = -l61;
l68 = -l63;
l69 = -l64;
l72 = l26+l27;
l73 = -l70;
l38 = -l33;
l43 = -l39;
l45 = -l36;
l74 = 1.0./l72;
l75 = l34+l37+l38+l40+l41+l42+l43+l44+l45+l46+l48+l51+l53+l54+l55+l56+l57+l59+l60+l62+l65+l66+l67+l68+l69+l71+l73;
l76 = l21.*l75;
l77 = -l76;
l78 = sqrt(complex(l77));
l79 = l19.*l78;
t2 = [(l74.*(-l22+l23+l24-l25-l28+l29+l30+l31-l32+l79))./6.0;l74.*(l22-l23-l24+l25+l28-l29-l30-l31+l32+l79).*(-1.0./6.0)];

l2 = A_min.^2;
l3 = A_max.^2;
l4 = A_wayp.^2;
t6 = (J_min.*l2-J_min.*l3-J_min.*l4-J_max.*l2+J_max.*l3+J_max.*l4+A_init.^2.*J_min-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_max.*2.0-A_max.*J_min.*J_max.*t2.*2.0)./(A_min.*J_min.*J_max.*2.0);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6];

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

