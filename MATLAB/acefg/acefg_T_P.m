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

function [t] = acefg_T_P(P_init,V_init,A_init,P_wayp,~,~,~,V_min,~,A_min,J_max,J_min,T) %#codegen
% Generated on 03-Sep-2019 15:34:01
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_init.^5;
l6 = A_min.^2;
l7 = A_min.^3;
l9 = A_min.^5;
l11 = J_min.^2;
l12 = J_min.^3;
l13 = J_max.^2;
l14 = J_max.^3;
l16 = J_max.^5;
l18 = J_max.^7;
l19 = T.^2;
l20 = V_init.^2;
l21 = V_min.^2;
l4 = l2.^2;
l8 = l6.^2;
l10 = l6.^3;
l15 = l13.^2;
l17 = l13.^3;
l22 = l15.^2;
coefficients = [J_min.*l22.*-3.0+l11.*l18.*3.0-l12.*l17+l14.^3,A_init.*l22.*6.0-A_min.*l22.*6.0-A_init.*J_min.*l18.*1.8e+1+A_min.*J_min.*l18.*1.8e+1+A_init.*l11.*l17.*1.8e+1-A_init.*l12.*l16.*6.0-A_min.*l11.*l17.*1.8e+1+A_min.*l12.*l16.*6.0,l2.*l18.*1.5e+1+l6.*l18.*1.5e+1-A_init.*A_min.*l18.*3.0e+1-J_min.*V_init.*l18.*6.0+J_min.*V_min.*l18.*6.0-J_min.*l2.*l17.*4.2e+1-J_min.*l6.*l17.*3.9e+1+V_init.*l11.*l17.*1.2e+1-V_init.*l12.*l16.*6.0-V_min.*l11.*l17.*1.2e+1+V_min.*l12.*l16.*6.0+l2.*l11.*l16.*3.9e+1-l2.*l12.*l15.*1.2e+1+l6.*l11.*l16.*3.3e+1-l6.*l12.*l15.*9.0+A_init.*A_min.*J_min.*l17.*8.4e+1-A_min.*J_min.*T.*l18.*6.0-A_init.*A_min.*l11.*l16.*7.8e+1+A_init.*A_min.*l12.*l15.*2.4e+1+A_min.*T.*l11.*l17.*1.2e+1-A_min.*T.*l12.*l16.*6.0,l3.*l17.*2.0e+1-l7.*l17.*2.0e+1+A_init.*l6.*l17.*6.0e+1-A_min.*l2.*l17.*6.0e+1-J_min.*l3.*l16.*4.8e+1+J_min.*l7.*l16.*4.4e+1+l3.*l11.*l15.*3.6e+1-l3.*l12.*l14.*8.0-l7.*l11.*l15.*3.6e+1+l7.*l12.*l14.*1.2e+1-T.*l6.*l11.*l16.*4.8e+1+T.*l6.*l12.*l15.*2.4e+1-A_init.*J_min.*V_init.*l17.*2.4e+1+A_init.*J_min.*V_min.*l17.*2.4e+1+A_min.*J_min.*V_init.*l17.*2.4e+1-A_min.*J_min.*V_min.*l17.*2.4e+1-A_init.*J_min.*l6.*l16.*1.32e+2+A_min.*J_min.*l2.*l16.*1.44e+2+A_init.*V_init.*l11.*l16.*4.8e+1-A_init.*V_init.*l12.*l15.*2.4e+1-A_init.*V_min.*l11.*l16.*4.8e+1+A_init.*V_min.*l12.*l15.*2.4e+1-A_min.*V_init.*l11.*l16.*4.8e+1+A_min.*V_init.*l12.*l15.*2.4e+1+A_min.*V_min.*l11.*l16.*4.8e+1-A_min.*V_min.*l12.*l15.*2.4e+1+J_min.*T.*l6.*l17.*2.4e+1+A_init.*l6.*l11.*l15.*8.4e+1-A_init.*l6.*l12.*l14.*1.2e+1-A_min.*l2.*l11.*l15.*1.08e+2+A_min.*l2.*l12.*l14.*2.4e+1+A_init.*A_min.*T.*l11.*l16.*4.8e+1-A_init.*A_min.*T.*l12.*l15.*2.4e+1-A_init.*A_min.*J_min.*T.*l17.*2.4e+1,l4.*l16.*1.5e+1+l8.*l16.*1.5e+1-A_init.*l7.*l16.*6.0e+1-A_min.*l3.*l16.*6.0e+1-J_min.*l4.*l15.*2.7e+1-J_min.*l8.*l15.*3.3e+1+l2.*l6.*l16.*9.0e+1+l4.*l11.*l14.*1.2e+1+l8.*l11.*l14.*3.3e+1-l8.*l12.*l13.*1.5e+1+l11.*l16.*l20.*1.2e+1-l12.*l15.*l20.*1.2e+1+l11.*l16.*l21.*1.2e+1-l12.*l15.*l21.*1.2e+1-J_min.*l2.*l6.*l15.*1.44e+2+T.*l7.*l11.*l15.*7.2e+1-T.*l7.*l12.*l14.*3.6e+1+V_init.*l2.*l11.*l15.*6.0e+1-V_init.*l2.*l12.*l14.*2.4e+1+V_init.*l6.*l11.*l15.*4.8e+1-V_init.*l6.*l12.*l14.*1.2e+1-V_min.*l2.*l11.*l15.*6.0e+1+V_min.*l2.*l12.*l14.*2.4e+1-V_min.*l6.*l11.*l15.*4.8e+1+V_min.*l6.*l12.*l14.*1.2e+1+l2.*l6.*l11.*l14.*4.2e+1+l2.*l6.*l12.*l13.*1.2e+1+l6.*l11.*l16.*l19.*1.2e+1-l6.*l12.*l15.*l19.*1.2e+1+A_init.*J_min.*l7.*l15.*9.6e+1+A_min.*J_min.*l3.*l15.*1.08e+2-J_min.*T.*l7.*l16.*3.6e+1-J_min.*V_init.*l2.*l16.*3.6e+1-J_min.*V_init.*l6.*l16.*3.6e+1+J_min.*V_min.*l2.*l16.*3.6e+1+J_min.*V_min.*l6.*l16.*3.6e+1-V_init.*V_min.*l11.*l16.*2.4e+1+V_init.*V_min.*l12.*l15.*2.4e+1-A_init.*l7.*l11.*l14.*3.6e+1-A_min.*l3.*l11.*l14.*4.8e+1-A_init.*A_min.*V_init.*l11.*l15.*1.2e+2+A_init.*A_min.*V_init.*l12.*l14.*4.8e+1+A_init.*A_min.*V_min.*l11.*l15.*1.2e+2-A_init.*A_min.*V_min.*l12.*l14.*4.8e+1+A_init.*J_min.*T.*l6.*l16.*7.2e+1-A_min.*J_min.*T.*l2.*l16.*3.6e+1+A_min.*T.*V_init.*l11.*l16.*2.4e+1-A_min.*T.*V_init.*l12.*l15.*2.4e+1-A_min.*T.*V_min.*l11.*l16.*2.4e+1+A_min.*T.*V_min.*l12.*l15.*2.4e+1-A_init.*T.*l6.*l11.*l15.*1.2e+2+A_init.*T.*l6.*l12.*l14.*4.8e+1+A_min.*T.*l2.*l11.*l15.*6.0e+1-A_min.*T.*l2.*l12.*l14.*2.4e+1+A_init.*A_min.*J_min.*V_init.*l16.*7.2e+1-A_init.*A_min.*J_min.*V_min.*l16.*7.2e+1,l5.*l15.*6.0-l9.*l15.*6.0+A_init.*l8.*l15.*3.0e+1-A_min.*l4.*l15.*3.0e+1-J_min.*l5.*l14.*6.0+J_min.*l9.*l14.*1.8e+1+J_max.*l9.*l12.*6.0-l2.*l7.*l15.*6.0e+1+l3.*l6.*l15.*6.0e+1-l9.*l11.*l13.*1.8e+1+J_min.*l2.*l7.*l14.*4.8e+1-J_min.*l3.*l6.*l14.*4.8e+1-T.*l8.*l11.*l14.*4.8e+1+T.*l8.*l12.*l13.*2.4e+1+V_init.*l3.*l11.*l14.*2.4e+1-V_init.*l7.*l12.*l13.*2.4e+1-V_min.*l3.*l11.*l14.*2.4e+1+V_min.*l7.*l12.*l13.*2.4e+1+l2.*l7.*l11.*l13.*1.2e+1-l3.*l6.*l11.*l13.*1.2e+1-l7.*l11.*l15.*l19.*2.4e+1+l7.*l12.*l14.*l19.*2.4e+1-A_init.*J_min.*l8.*l14.*4.2e+1-A_init.*J_max.*l8.*l12.*6.0+A_min.*J_min.*l4.*l14.*3.0e+1+J_min.*T.*l8.*l15.*2.4e+1-J_min.*V_init.*l3.*l15.*2.4e+1+J_min.*V_init.*l7.*l15.*2.4e+1+J_min.*V_min.*l3.*l15.*2.4e+1-J_min.*V_min.*l7.*l15.*2.4e+1+A_init.*l8.*l11.*l13.*1.8e+1+A_init.*l11.*l15.*l20.*2.4e+1-A_init.*l12.*l14.*l20.*2.4e+1+A_init.*l11.*l15.*l21.*2.4e+1-A_init.*l12.*l14.*l21.*2.4e+1-A_min.*l11.*l15.*l20.*2.4e+1+A_min.*l12.*l14.*l20.*2.4e+1-A_min.*l11.*l15.*l21.*2.4e+1+A_min.*l12.*l14.*l21.*2.4e+1-A_init.*J_min.*T.*l7.*l15.*7.2e+1-A_min.*J_min.*T.*l3.*l15.*2.4e+1-A_init.*J_min.*V_init.*l6.*l15.*7.2e+1+A_min.*J_min.*V_init.*l2.*l15.*7.2e+1+A_init.*J_min.*V_min.*l6.*l15.*7.2e+1-A_min.*J_min.*V_min.*l2.*l15.*7.2e+1-A_init.*V_init.*V_min.*l11.*l15.*4.8e+1+A_init.*V_init.*V_min.*l12.*l14.*4.8e+1+A_min.*V_init.*V_min.*l11.*l15.*4.8e+1-A_min.*V_init.*V_min.*l12.*l14.*4.8e+1+A_init.*T.*l7.*l11.*l14.*9.6e+1-A_init.*T.*l7.*l12.*l13.*2.4e+1+A_min.*T.*l3.*l11.*l14.*2.4e+1+A_init.*V_init.*l6.*l11.*l14.*4.8e+1+A_init.*V_init.*l6.*l12.*l13.*2.4e+1-A_min.*V_init.*l2.*l11.*l14.*7.2e+1-A_init.*V_min.*l6.*l11.*l14.*4.8e+1-A_init.*V_min.*l6.*l12.*l13.*2.4e+1+A_min.*V_min.*l2.*l11.*l14.*7.2e+1+J_min.*T.*l2.*l6.*l15.*7.2e+1-T.*V_init.*l6.*l11.*l15.*4.8e+1+T.*V_init.*l6.*l12.*l14.*4.8e+1+T.*V_min.*l6.*l11.*l15.*4.8e+1-T.*V_min.*l6.*l12.*l14.*4.8e+1+A_init.*l6.*l11.*l15.*l19.*2.4e+1-A_init.*l6.*l12.*l14.*l19.*2.4e+1-T.*l2.*l6.*l11.*l14.*7.2e+1+A_init.*A_min.*T.*V_init.*l11.*l15.*4.8e+1-A_init.*A_min.*T.*V_init.*l12.*l14.*4.8e+1-A_init.*A_min.*T.*V_min.*l11.*l15.*4.8e+1+A_init.*A_min.*T.*V_min.*l12.*l14.*4.8e+1,l10.*l12+l10.*l14+l2.^3.*l14-A_init.*l9.*l14.*6.0-A_min.*l5.*l14.*6.0-J_min.*l10.*l13.*5.0+J_max.*l10.*l11.*3.0+l2.*l8.*l14.*1.5e+1-l3.*l7.*l14.*2.0e+1+l4.*l6.*l14.*1.5e+1-V_init.^3.*l12.*l14.*8.0+V_min.^3.*l12.*l14.*8.0-J_min.*l2.*l8.*l13.*6.0-J_min.*l3.*l7.*l13.*4.0+J_min.*l4.*l6.*l13.*3.0+J_max.*l2.*l8.*l11.*3.0-P_init.*l7.*l12.*l13.*4.8e+1+P_wayp.*l7.*l12.*l13.*4.8e+1+T.*l9.*l11.*l13.*1.2e+1-V_init.*l8.*l11.*l13.*1.2e+1-V_init.*l12.*l14.*l21.*2.4e+1+V_min.*l8.*l11.*l13.*1.2e+1+V_min.*l12.*l14.*l20.*2.4e+1+l2.*l11.*l14.*l20.*1.2e+1+l2.*l11.*l14.*l21.*1.2e+1+l6.*l11.*l14.*l20.*1.2e+1+l6.*l12.*l13.*l20.*1.2e+1+l6.*l11.*l14.*l21.*1.2e+1+l6.*l12.*l13.*l21.*1.2e+1+l8.*l11.*l14.*l19.*1.2e+1-l8.*l12.*l13.*l19.*1.2e+1-T.^3.*l7.*l12.*l14.*8.0+A_init.*J_min.*l9.*l13.*1.2e+1-A_init.*J_max.*l9.*l11.*6.0-J_min.*T.*l9.*l14.*6.0-J_max.*T.*l9.*l12.*6.0-J_min.*V_init.*l4.*l14.*6.0-J_min.*V_init.*l8.*l14.*6.0-J_max.*V_init.*l8.*l12.*6.0+J_min.*V_min.*l4.*l14.*6.0+J_min.*V_min.*l8.*l14.*6.0+J_max.*V_min.*l8.*l12.*6.0+A_init.*J_min.*T.*l8.*l14.*2.4e+1-A_min.*J_min.*T.*l4.*l14.*6.0+A_init.*J_min.*V_init.*l7.*l14.*2.4e+1+A_min.*J_min.*V_init.*l3.*l14.*2.4e+1-A_init.*J_min.*V_min.*l7.*l14.*2.4e+1-A_min.*J_min.*V_min.*l3.*l14.*2.4e+1-A_init.*A_min.*l11.*l14.*l20.*2.4e+1-A_init.*A_min.*l11.*l14.*l21.*2.4e+1-A_init.*T.*l8.*l11.*l13.*2.4e+1-A_min.*T.*l12.*l14.*l20.*2.4e+1-A_min.*T.*l12.*l14.*l21.*2.4e+1+A_init.*V_init.*l7.*l11.*l13.*2.4e+1-A_init.*V_min.*l7.*l11.*l13.*2.4e+1-J_min.*T.*l2.*l7.*l14.*3.6e+1+J_min.*T.*l3.*l6.*l14.*2.4e+1-J_min.*V_init.*l2.*l6.*l14.*3.6e+1+J_min.*V_min.*l2.*l6.*l14.*3.6e+1+T.*V_init.*l7.*l11.*l14.*2.4e+1-T.*V_init.*l7.*l12.*l13.*2.4e+1-T.*V_min.*l7.*l11.*l14.*2.4e+1-T.*V_min.*l7.*l12.*l13.*2.4e+1-V_init.*V_min.*l2.*l11.*l14.*2.4e+1-V_init.*V_min.*l6.*l11.*l14.*2.4e+1-V_init.*V_min.*l6.*l12.*l13.*2.4e+1-A_init.*l7.*l11.*l14.*l19.*2.4e+1+T.*l2.*l7.*l11.*l13.*1.2e+1-V_init.*l2.*l6.*l11.*l13.*1.2e+1-V_init.*l6.*l12.*l14.*l19.*2.4e+1+V_min.*l2.*l6.*l11.*l13.*1.2e+1+V_min.*l6.*l12.*l14.*l19.*2.4e+1+l2.*l6.*l11.*l14.*l19.*1.2e+1+A_init.*A_min.*V_init.*V_min.*l11.*l14.*4.8e+1+A_min.*T.*V_init.*V_min.*l12.*l14.*4.8e+1-A_init.*T.*V_init.*l6.*l11.*l14.*4.8e+1+A_min.*T.*V_init.*l2.*l11.*l14.*2.4e+1+A_init.*T.*V_min.*l6.*l11.*l14.*4.8e+1-A_min.*T.*V_min.*l2.*l11.*l14.*2.4e+1];
 
t1 = roots(coefficients);
l2 = J_max.*t1;
l3 = A_min.^2;
t7 = ((J_min.*l3+J_max.*l3.*2.0-J_max.*(l3+J_min.*V_init.*2.0+A_init.*l2.*2.0+A_init.^2+l2.^2+A_min.*J_min.*T.*2.0+A_init.*J_min.*t1.*2.0+J_min.*l2.*t1)+J_min.*J_max.*V_min.*2.0+A_min.*J_min.*l2.*2.0+J_max.*(A_init+l2).*(A_init-A_min+l2).*2.0).*(-1.0./2.0))./(A_min.*J_min.*J_max);

l2 = J_max.*t1;
l3 = J_max.^2;
l4 = -A_min;
l5 = A_init+l2+l4;
t6 = ((J_min.*l2.^2+J_max.*l5.^2-J_min.*(A_min+J_max.*t7).^2-J_max.*l5.*(A_init+l2).*2.0+J_min.*J_max.*V_init.*2.0-J_min.*J_max.*V_min.*2.0+A_init.*J_min.*l2.*2.0+J_min.*l3.*t7.^2+A_min.*J_min.*J_max.*t7.*2.0).*(-1.0./2.0))./(A_min.*J_min.*J_max);

t3 = -(A_init-A_min+J_max.*t1)./J_min;

t5 = [0.0;0.0;0.0;0.0;0.0;0.0];

t4 = [0.0;0.0;0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end

