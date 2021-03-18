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

function [t] = abc_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,~,~,~,A_max,~,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 17:25:45
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_max.^2;
l6 = A_max.^3;
l8 = J_max.^2;
l9 = V_init.^2;
l10 = V_wayp.^2;
l11 = 1.0./J_min;
l17 = sqrt(3.0);
l18 = sqrt(6.0);
l22 = A_init.*A_max.*J_max.*V_init.*2.4e+1;
l4 = l2.^2;
l7 = l5.^2;
l12 = l11.^2;
l13 = l11.^3;
l15 = 1.0./l8;
l20 = A_max.*l3.*8.0;
l21 = V_wayp.*l11.*4.0;
l23 = -l22;
l24 = A_max.*P_init.*l8.*2.4e+1;
l25 = A_max.*P_wayp.*l8.*2.4e+1;
l26 = J_max.*V_init.*l2.*1.2e+1;
l27 = J_max.*V_init.*l5.*1.2e+1;
l28 = l2.*l5.*6.0;
l29 = (A_max.*l11)./3.0;
l32 = l8.*l9.*1.2e+1;
l33 = l8.*l10.*1.2e+1;
l14 = l12.^2;
l16 = l4.*3.0;
l30 = -l25;
l31 = -l28;
l34 = -l29;
l35 = -l32;
l36 = A_max.*V_wayp.*l12.*(8.0./3.0);
l37 = l5.*l12.*(2.0./3.0);
l40 = V_wayp.*l5.*l13.*(4.0./9.0);
l41 = V_wayp.*l5.*l13.*(1.6e+1./3.0);
l43 = l6.*l13.*(8.0./2.7e+1);
l19 = -l16;
l38 = l7.*l14.*(4.0./9.0);
l39 = (l7.*l14)./2.7e+1;
l44 = -l41;
l45 = l21+l37;
l51 = l36+l43;
l42 = -l38;
l46 = l45.^2;
l47 = l45.^3;
l53 = l51.^2;
l61 = l7+l19+l20+l23+l24+l26+l27+l30+l31+l33+l35;
l48 = l46.^2;
l49 = l47.*2.0;
l52 = l47./2.7e+1;
l54 = l53.^2;
l56 = l53.*2.7e+1;
l58 = l53./2.0;
l59 = l47.*l53.*4.0;
l62 = l12.*l15.*l61.*4.0;
l63 = (l12.*l15.*l61)./3.0;
l50 = -l49;
l55 = -l52;
l57 = l54.*2.7e+1;
l60 = -l59;
l64 = -l63;
l65 = l39+l40+l64;
l66 = l65.^2;
l67 = l65.^3;
l69 = l45.*l65.*(4.0./3.0);
l70 = l45.*l65.*7.2e+1;
l71 = l48.*l65.*1.6e+1;
l75 = l45.*l53.*l65.*1.44e+2;
l68 = l67.*2.56e+2;
l72 = -l69;
l73 = -l70;
l74 = l46.*l66.*1.28e+2;
l76 = -l75;
l77 = l57+l60+l68+l71+l74+l76;
l78 = sqrt(complex(l77));
l79 = l17.*l78.*3.0;
l80 = (l17.*l78)./1.8e+1;
l81 = l50+l56+l73+l79;
l83 = l55+l58+l72+l80;
l82 = sqrt(complex(l81));
l84 = l83.^(1.0./3.0);
l86 = 1.0./l83.^(1.0./6.0);
l85 = l84.^2;
l88 = l45.*l84.*6.0;
l89 = l18.*l51.*l82.*3.0;
l87 = l85.*9.0;
l90 = -l89;
l91 = l42+l44+l46+l62+l87+l88;
l92 = sqrt(complex(l91));
l93 = 1.0./l91.^(1.0./4.0);
l94 = l46.*l92;
l96 = l65.*l92.*1.2e+1;
l98 = l85.*l92.*-9.0;
l99 = (l86.*l92)./6.0;
l101 = l45.*l84.*l92.*1.2e+1;
l95 = -l94;
l100 = -l99;
l102 = l89+l95+l96+l98+l101;
l103 = l90+l95+l96+l98+l101;
l104 = sqrt(complex(l102));
l105 = sqrt(complex(l103));
l106 = (l86.*l93.*l104)./6.0;
l107 = (l86.*l93.*l105)./6.0;
t3 = [l34+l100-l106;l34+l100+l106;l34+l99-l107;l34+l99+l107];

t2 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0-A_init.^2+A_max.^2+A_max.*J_max.*t3.*2.0+J_min.*J_max.*t3.^2).*(-1.0./2.0))./(A_max.*J_max);

l2 = -A_max;
l3 = 1.0./J_max;
l4 = A_init+l2;
l5 = l3.*l4;
l6 = -l5;
t1 = [l6;l6;l6;l6];

t7 = [0.0;0.0;0.0;0.0];

t6 = [0.0;0.0;0.0;0.0];

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

