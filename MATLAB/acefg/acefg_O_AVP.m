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

function [t] = acefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,A_min,J_max,J_min) %#codegen
% Generated on 28-Aug-2019 12:21:18
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_min.^2;
l6 = A_wayp.^2;
l7 = A_wayp.^3;
l9 = J_min.^2;
l10 = J_min.^3;
l12 = J_max.^2;
l13 = V_init.^2;
l14 = V_wayp.^2;
l18 = sqrt(3.0);
l19 = sqrt(6.0);
l26 = A_init.*A_min.*J_max.*V_init.*2.4e+1;
l27 = A_min.*A_wayp.*J_max.*V_wayp.*2.4e+1;
l4 = l2.^2;
l8 = l6.^2;
l11 = l9.^2;
l21 = J_max.*l10.*6.0;
l22 = A_min.*l3.*8.0;
l23 = A_min.*l7.*8.0;
l24 = A_min.*l10.*8.0;
l28 = J_min.*J_max.*l2.*6.0;
l29 = A_min.*J_min.*l12.*4.0;
l30 = J_min.*J_max.*l5.*6.0;
l34 = A_min.*J_max.*l9.*1.2e+1;
l35 = A_min.*P_init.*l12.*2.4e+1;
l36 = A_min.*P_wayp.*l12.*2.4e+1;
l37 = J_max.*V_init.*l2.*1.2e+1;
l38 = J_max.*V_init.*l5.*1.2e+1;
l39 = J_max.*V_wayp.*l5.*1.2e+1;
l40 = J_max.*V_wayp.*l6.*1.2e+1;
l41 = l2.*l5.*6.0;
l42 = l5.*l6.*6.0;
l43 = J_min.*V_init.*l12.*1.2e+1;
l44 = J_max.*V_init.*l9.*1.2e+1;
l45 = l2.*l9.*6.0;
l46 = l5.*l9.*6.0;
l47 = l9.*l12.*3.0;
l55 = l12.*l13.*1.2e+1;
l56 = l12.*l14.*1.2e+1;
l15 = l4.*3.0;
l16 = l8.*3.0;
l17 = l11.*3.0;
l25 = -l21;
l48 = -l34;
l58 = l17+l25+l47;
l63 = l24+l29+l48;
l59 = 1.0./l58;
l64 = l63.^2;
l65 = l63.^3;
l60 = l59.^2;
l61 = l59.^3;
l66 = l64.^2;
l68 = (l59.*l63)./4.0;
l74 = -l59.*(l28-l30-l43+l44-l45+l46);
l90 = l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56).*1.2e+1;
l62 = l60.^2;
l69 = l60.*l64.*(3.0./8.0);
l70 = (l61.*l65)./8.0;
l75 = l60.*l63.*(l28-l30-l43+l44-l45+l46).*(-1.0./2.0);
l77 = l61.*l64.*(l28-l30-l43+l44-l45+l46).*(3.0./4.0);
l71 = l62.*l66.*(3.0./2.56e+2);
l72 = l62.*l66.*(9.0./6.4e+1);
l80 = l69+l74;
l91 = l70+l75;
l73 = -l72;
l81 = l80.^2;
l82 = l80.^3;
l92 = l91.^2;
l100 = (-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).^2;
l102 = (-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).^3.*-2.56e+2;
l105 = l80.*(-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).*(4.0./3.0);
l106 = l80.*(-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).*7.2e+1;
l83 = l81.^2;
l84 = l82.*2.0;
l86 = l82./2.7e+1;
l93 = l92.^2;
l94 = l92.*2.7e+1;
l96 = l92./2.0;
l97 = l82.*l92.*4.0;
l108 = l81.*l100.*1.28e+2;
l110 = l80.*l92.*(-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).*1.44e+2;
l85 = -l84;
l87 = -l86;
l95 = l93.*2.7e+1;
l98 = -l97;
l107 = l83.*(-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).*-1.6e+1;
l111 = l95+l98+l102+l107+l108+l110;
l112 = sqrt(complex(l111));
l113 = l18.*l112.*3.0;
l114 = (l18.*l112)./1.8e+1;
l115 = l85+l94+l106+l113;
l117 = l87+l96+l105+l114;
l116 = sqrt(complex(l115));
l118 = l117.^(1.0./3.0);
l120 = 1.0./l117.^(1.0./6.0);
l119 = l118.^2;
l122 = l80.*l118.*6.0;
l123 = l19.*l91.*l116.*3.0;
l121 = l119.*9.0;
l124 = -l123;
l125 = l73+l77+l81+l90+l121+l122;
l126 = sqrt(complex(l125));
l127 = 1.0./l125.^(1.0./4.0);
l128 = l81.*l126;
l130 = l126.*(-l71+l59.*(l15-l16-l22+l23+l26-l27-l35+l36-l37-l38+l39+l40+l41-l42+l55-l56)+(l61.*l64.*(l28-l30-l43+l44-l45+l46))./1.6e+1).*-1.2e+1;
l132 = l119.*l126.*-9.0;
l133 = (l120.*l126)./6.0;
l135 = l80.*l118.*l126.*1.2e+1;
l129 = -l128;
l134 = -l133;
l136 = l123+l129+l130+l132+l135;
l137 = l124+l129+l130+l132+l135;
l138 = sqrt(complex(l136));
l139 = sqrt(complex(l137));
l140 = (l120.*l127.*l138)./6.0;
l141 = (l120.*l127.*l139)./6.0;
t3 = [l68+l134-l141;l68+l134+l141;l68+l133-l140;l68+l133+l140];

l2 = t3.^2;
t6 = ((J_max.*V_init.*2.0-J_max.*V_wayp.*2.0+J_min.^2.*l2-A_init.^2+A_wayp.^2-A_min.*J_min.*t3.*2.0+A_min.*J_max.*t3.*2.0-J_min.*J_max.*l2).*(-1.0./2.0))./(A_min.*J_max);

l2 = -A_wayp;
l3 = 1.0./J_max;
l4 = A_min+l2;
l5 = l3.*l4;
l6 = -l5;
t7 = [l6;l6;l6;l6];

t1 = -(A_init-A_min+J_min.*t3)./J_max;

t5 = [0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end


