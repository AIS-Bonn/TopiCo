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

function [t] = acefg_T_AP(P_init,V_init,A_init,P_wayp,~,A_wayp,~,~,~,A_min,J_max,J_min,T) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l4 = A_min.^2;
l5 = A_min.^3;
l6 = A_wayp.^2;
l7 = A_wayp.^3;
l8 = J_min.^2;
l9 = J_max.^2;
l10 = J_max.^3;
l12 = J_max.^5;
l13 = T.^2;
l14 = sqrt(3.0);
l11 = l9.^2;
l20 = A_init.*A_min.*l10.*6.0;
l21 = A_init.*J_min.*l10.*6.0;
l22 = A_min.*J_min.*l10.*6.0;
l24 = l3.*l9;
l25 = l5.*l8;
l26 = l5.*l9;
l27 = l7.*l8;
l30 = l2.*l10.*3.0;
l31 = l4.*l10.*3.0;
l32 = l8.*l10.*2.0;
l33 = A_init.*A_min.*J_min.*l9.*6.0;
l36 = P_init.*l8.*l9.*6.0;
l37 = P_wayp.*l8.*l9.*6.0;
l38 = T.*l8.*l10.*3.0;
l43 = A_init.*l4.*l9.*3.0;
l44 = A_min.*l2.*l9.*3.0;
l45 = A_min.*l6.*l8.*3.0;
l46 = A_wayp.*l4.*l8.*3.0;
l47 = A_init.*l8.*l9.*3.0;
l48 = J_min.*l2.*l9.*3.0;
l49 = A_min.*l8.*l9.*3.0;
l50 = J_min.*l4.*l9.*3.0;
l54 = A_init.*T.*l8.*l9.*6.0;
l56 = A_min.*T.*l8.*l9.*6.0;
l57 = T.*V_init.*l8.*l9.*6.0;
l63 = J_min.*T.*l2.*l9.*-3.0;
l65 = J_min.*T.*l4.*l9.*-3.0;
l15 = A_init.*l11.*3.0;
l16 = A_min.*l11.*3.0;
l17 = J_min.*l11.*3.0;
l29 = -l21;
l35 = T.*l22;
l39 = T.*l33;
l40 = -l24;
l41 = -l25;
l51 = -l37;
l52 = -l38;
l58 = -l43;
l59 = -l45;
l61 = -l49;
l66 = l13.*l49;
l18 = -l16;
l19 = -l17;
l23 = T.*l17;
l82 = l26+l27+l36+l39+l40+l41+l44+l46+l51+l57+l58+l59+l63+l65+l66;
l67 = l12+l19+l32;
l71 = l15+l18+l22+l23+l29+l47+l52+l61;
l68 = 1.0./l67;
l72 = l71.^2;
l73 = l71.^3;
l69 = l68.^2;
l70 = l68.^3;
l75 = (l68.*l71)./3.0;
l83 = (l68.*l82)./2.0;
l76 = -l75;
l77 = (l69.*l72)./9.0;
l78 = (l70.*l73)./2.7e+1;
l84 = l69.*l71.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0).*(-1.0./6.0);
l80 = -l78;
l86 = -(l77+(l68.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0))./3.0).^3;
l88 = (l78-l83+(l69.*l71.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0))./6.0).^2;
l87 = l80+l83+l84;
l89 = l86+l88;
l90 = sqrt(complex(l89));
l91 = l87+l90;
l92 = l91.^(1.0./3.0);
l93 = 1.0./l92;
l94 = l92./2.0;
l95 = -l94;
l96 = -l93.*(l77+(l68.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0))./3.0);
l97 = l93.*(l77+(l68.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0))./3.0).*(-1.0./2.0);
l98 = l92+l96;
l99 = l14.*l98.*5.0e-1i;
t1 = [l76+l92+l93.*(l77+(l68.*(l20-l30-l31-l33+l35+l48+l50+l54-l56-A_init.*J_min.*T.*l10.*6.0))./3.0);l76+l95+l97-l99;l76+l95+l97+l99];

t6 = (A_init-A_min-J_min.*t1+J_max.*t1+J_min.*T+(J_min.*(A_min-A_wayp))./J_max)./J_min;

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6;l6];

t3 = -(A_init-A_min+J_max.*t1)./J_min;

t2 = [0.0;0.0;0.0];

t5 = [0.0;0.0;0.0];

t4 = [0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


