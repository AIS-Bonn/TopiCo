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

function [t] = acdeg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,~,~,~,~,J_max,J_min,T) %#codegen
% Generated on 29-Aug-2019 10:09:55
coder.inline('default');

l2 = A_init.^2;
l3 = A_init.^3;
l5 = A_wayp.^2;
l6 = A_wayp.^3;
l8 = J_min.^2;
l9 = J_min.^3;
l10 = J_max.^2;
l11 = J_max.^3;
l13 = T.^2;
l14 = T.^3;
l16 = T.^5;
l17 = V_init.^2;
l18 = V_wayp.^2;
l4 = l2.^2;
l7 = l5.^2;
l12 = l10.^2;
l15 = l13.^2;
coefficients = [-l8.*l12+l9.*l11.*2.0,A_init.*l8.*l11.*2.0-A_init.*l9.*l10.*7.0-A_wayp.*l8.*l11.*2.0+A_wayp.*l9.*l10.*7.0+T.*l8.*l12.*2.0-T.*l9.*l11.*7.0,J_max.*l2.*l9.*8.0+J_max.*l5.*l9.*8.0+l2.*l8.*l10.*2.0+l5.*l8.*l10.*2.0+l8.*l12.*l13.*2.0+l9.*l11.*l13.*8.0-A_init.*A_wayp.*J_max.*l9.*1.6e+1-A_init.*A_wayp.*l8.*l10.*4.0+A_init.*T.*l8.*l11.*4.0+A_init.*T.*l9.*l10.*1.6e+1-A_wayp.*T.*l8.*l11.*4.0-A_wayp.*T.*l9.*l10.*1.6e+1,J_min.*P_init.*l12.*2.4e+1-J_min.*P_wayp.*l12.*2.4e+1-A_init.*l5.*l9.*1.2e+1+A_wayp.*l2.*l9.*1.2e+1+J_min.*l3.*l10.*2.0-J_max.*l3.*l8.*1.2e+1-J_min.*l6.*l10.*2.0+J_max.*l6.*l8.*1.2e+1-P_init.*l8.*l11.*4.8e+1+P_init.*l9.*l10.*2.4e+1+P_wayp.*l8.*l11.*4.8e+1-P_wayp.*l9.*l10.*2.4e+1-l8.*l12.*l14.*8.0-l9.*l11.*l14.*2.0-T.*l2.*l8.*l10.*1.2e+1-T.*l5.*l8.*l10.*1.2e+1-A_init.*J_min.*V_init.*l11.*1.2e+1-A_init.*J_max.*V_init.*l9.*1.2e+1+A_init.*J_min.*V_wayp.*l11.*1.2e+1+A_init.*J_max.*V_wayp.*l9.*1.2e+1-A_wayp.*J_min.*V_init.*l11.*1.2e+1-A_wayp.*J_max.*V_init.*l9.*1.2e+1+A_wayp.*J_min.*V_wayp.*l11.*1.2e+1+A_wayp.*J_max.*V_wayp.*l9.*1.2e+1+J_min.*T.*V_init.*l12.*1.2e+1+J_min.*T.*V_wayp.*l12.*1.2e+1-A_init.*J_min.*l5.*l10.*6.0-A_init.*J_max.*l5.*l8.*1.2e+1+A_wayp.*J_min.*l2.*l10.*6.0+A_wayp.*J_max.*l2.*l8.*1.2e+1+A_init.*V_init.*l8.*l10.*2.4e+1-A_init.*V_wayp.*l8.*l10.*2.4e+1+A_wayp.*V_init.*l8.*l10.*2.4e+1-A_wayp.*V_wayp.*l8.*l10.*2.4e+1-J_min.*T.*l2.*l11.*6.0-J_max.*T.*l2.*l9.*1.2e+1-J_min.*T.*l5.*l11.*6.0-J_max.*T.*l5.*l9.*1.2e+1-T.*V_init.*l8.*l11.*2.4e+1+T.*V_init.*l9.*l10.*1.2e+1-T.*V_wayp.*l8.*l11.*2.4e+1+T.*V_wayp.*l9.*l10.*1.2e+1-A_init.*l8.*l11.*l13.*2.4e+1-A_init.*l9.*l10.*l13.*6.0+A_wayp.*l8.*l11.*l13.*2.4e+1+A_wayp.*l9.*l10.*l13.*6.0+A_init.*A_wayp.*T.*l8.*l10.*4.8e+1+A_init.*A_wayp.*J_max.*T.*l9.*1.2e+1,l4.*l10.*-3.0-l7.*l10.*3.0-l12.*l17.*1.2e+1-l12.*l18.*1.2e+1+J_min.*J_max.*l4.*8.0+J_min.*J_max.*l7.*8.0+V_init.*V_wayp.*l12.*2.4e+1-A_init.*l6.*l8.*1.2e+1-A_wayp.*l3.*l8.*1.2e+1+J_min.*l11.*l17.*4.8e+1+J_max.*l9.*l17.*2.4e+1+J_min.*l11.*l18.*4.8e+1+J_max.*l9.*l18.*2.4e+1+V_init.*l2.*l11.*1.2e+1-V_init.*l5.*l11.*1.2e+1-V_wayp.*l2.*l11.*1.2e+1+V_wayp.*l5.*l11.*1.2e+1+l2.*l5.*l8.*2.4e+1+l2.*l5.*l10.*6.0-l8.*l10.*l17.*6.0e+1+l8.*l12.*l15.*7.0-l9.*l11.*l15.*2.0-l8.*l10.*l18.*6.0e+1+J_min.*l2.*l11.*l13.*1.2e+1+J_min.*l5.*l11.*l13.*1.2e+1+V_init.*l8.*l11.*l13.*4.8e+1-V_init.*l9.*l10.*l13.*2.4e+1+V_wayp.*l8.*l11.*l13.*4.8e+1-V_wayp.*l9.*l10.*l13.*2.4e+1+l2.*l8.*l10.*l13.*1.8e+1+l5.*l8.*l10.*l13.*1.8e+1-A_init.*J_min.*J_max.*l6.*8.0-A_wayp.*J_min.*J_max.*l3.*8.0-A_init.*J_min.*P_init.*l11.*4.8e+1-A_init.*J_max.*P_init.*l9.*4.8e+1+A_init.*J_min.*P_wayp.*l11.*4.8e+1+A_init.*J_max.*P_wayp.*l9.*4.8e+1+A_wayp.*J_min.*P_init.*l11.*4.8e+1+A_wayp.*J_max.*P_init.*l9.*4.8e+1-A_wayp.*J_min.*P_wayp.*l11.*4.8e+1-A_wayp.*J_max.*P_wayp.*l9.*4.8e+1-J_min.*P_init.*T.*l12.*4.8e+1+J_min.*P_wayp.*T.*l12.*4.8e+1-J_min.*V_init.*V_wayp.*l11.*9.6e+1-J_max.*V_init.*V_wayp.*l9.*4.8e+1+A_init.*P_init.*l8.*l10.*9.6e+1-A_init.*P_wayp.*l8.*l10.*9.6e+1-A_wayp.*P_init.*l8.*l10.*9.6e+1+A_wayp.*P_wayp.*l8.*l10.*9.6e+1+J_min.*T.*l3.*l10.*8.0+J_max.*T.*l3.*l8.*1.2e+1-J_min.*T.*l6.*l10.*8.0-J_max.*T.*l6.*l8.*1.2e+1-J_min.*V_init.*l2.*l10.*2.4e+1+J_max.*V_init.*l2.*l8.*1.2e+1+J_min.*V_init.*l5.*l10.*2.4e+1-J_max.*V_init.*l5.*l8.*1.2e+1-J_min.*V_init.*l12.*l13.*2.4e+1+J_min.*V_wayp.*l2.*l10.*2.4e+1-J_max.*V_wayp.*l2.*l8.*1.2e+1-J_min.*V_wayp.*l5.*l10.*2.4e+1+J_max.*V_wayp.*l5.*l8.*1.2e+1-J_min.*V_wayp.*l12.*l13.*2.4e+1+P_init.*T.*l8.*l11.*9.6e+1-P_init.*T.*l9.*l10.*4.8e+1-P_wayp.*T.*l8.*l11.*9.6e+1+P_wayp.*T.*l9.*l10.*4.8e+1+V_init.*V_wayp.*l8.*l10.*1.2e+2+A_init.*l8.*l11.*l14.*2.8e+1-A_init.*l9.*l10.*l14.*8.0-A_wayp.*l8.*l11.*l14.*2.8e+1+A_wayp.*l9.*l10.*l14.*8.0-A_init.*J_min.*T.*V_wayp.*l11.*4.8e+1-A_init.*J_max.*T.*V_wayp.*l9.*4.8e+1+A_wayp.*J_min.*T.*V_init.*l11.*4.8e+1+A_wayp.*J_max.*T.*V_init.*l9.*4.8e+1+A_init.*A_wayp.*J_max.*l9.*l13.*2.4e+1+A_init.*J_min.*T.*l5.*l10.*2.4e+1+A_init.*J_max.*T.*l5.*l8.*3.6e+1-A_wayp.*J_min.*T.*l2.*l10.*2.4e+1-A_wayp.*J_max.*T.*l2.*l8.*3.6e+1+A_init.*T.*V_wayp.*l8.*l10.*9.6e+1-A_wayp.*T.*V_init.*l8.*l10.*9.6e+1-A_init.*A_wayp.*l8.*l10.*l13.*8.4e+1,-A_init.^5.*J_min+A_wayp.^5.*J_min-A_init.*J_min.*l7.*5.0+A_wayp.*J_min.*l4.*5.0-A_init.*l9.*l17.*1.2e+1-A_init.*l9.*l18.*1.2e+1+A_wayp.*l9.*l17.*1.2e+1+A_wayp.*l9.*l18.*1.2e+1+J_min.*l2.*l6.*1.0e+1-J_min.*l3.*l5.*1.0e+1+P_init.*l2.*l9.*2.4e+1+P_init.*l5.*l9.*2.4e+1-P_wayp.*l2.*l9.*2.4e+1-P_wayp.*l5.*l9.*2.4e+1-l8.*l12.*l16.*2.0+l9.*l11.*l16-J_min.*l3.*l10.*l13.*1.0e+1-J_min.*l2.*l11.*l14.*6.0+J_max.*l2.*l9.*l14.*4.0+J_min.*l6.*l10.*l13.*1.0e+1-J_min.*l5.*l11.*l14.*6.0+J_max.*l5.*l9.*l14.*4.0-P_init.*l8.*l11.*l13.*4.8e+1+P_init.*l9.*l10.*l13.*2.4e+1+P_wayp.*l8.*l11.*l13.*4.8e+1-P_wayp.*l9.*l10.*l13.*2.4e+1+T.*l8.*l10.*l17.*2.4e+1+T.*l8.*l10.*l18.*2.4e+1-V_init.*l8.*l11.*l14.*2.4e+1+V_init.*l9.*l10.*l14.*1.2e+1-V_wayp.*l8.*l11.*l14.*2.4e+1+V_wayp.*l9.*l10.*l14.*1.2e+1-l2.*l8.*l10.*l14.*8.0-l5.*l8.*l10.*l14.*8.0-A_init.*A_wayp.*P_init.*l9.*4.8e+1+A_init.*A_wayp.*P_wayp.*l9.*4.8e+1-J_min.*J_max.*T.*l4.*5.0-J_min.*J_max.*T.*l7.*5.0+A_init.*V_init.*V_wayp.*l9.*2.4e+1-A_wayp.*V_init.*V_wayp.*l9.*2.4e+1-A_init.*J_min.*l10.*l17.*1.2e+1+A_init.*J_max.*l8.*l17.*2.4e+1-A_init.*J_min.*l10.*l18.*1.2e+1+A_init.*J_max.*l8.*l18.*2.4e+1+A_wayp.*J_min.*l10.*l17.*1.2e+1-A_wayp.*J_max.*l8.*l17.*2.4e+1+A_wayp.*J_min.*l10.*l18.*1.2e+1-A_wayp.*J_max.*l8.*l18.*2.4e+1+J_min.*P_init.*l2.*l10.*2.4e+1-J_max.*P_init.*l2.*l8.*4.8e+1+J_min.*P_init.*l5.*l10.*2.4e+1-J_max.*P_init.*l5.*l8.*4.8e+1+J_min.*P_init.*l12.*l13.*2.4e+1-J_min.*P_wayp.*l2.*l10.*2.4e+1+J_max.*P_wayp.*l2.*l8.*4.8e+1-J_min.*P_wayp.*l5.*l10.*2.4e+1+J_max.*P_wayp.*l5.*l8.*4.8e+1-J_min.*P_wayp.*l12.*l13.*2.4e+1-J_min.*T.*l11.*l17.*1.2e+1-J_max.*T.*l9.*l17.*1.2e+1-J_min.*T.*l11.*l18.*1.2e+1-J_max.*T.*l9.*l18.*1.2e+1+J_min.*V_init.*l12.*l14.*1.2e+1+J_min.*V_wayp.*l12.*l14.*1.2e+1+T.*V_init.*l5.*l9.*2.4e+1+T.*V_wayp.*l2.*l9.*2.4e+1+A_init.*l5.*l9.*l13.*1.2e+1-A_init.*l8.*l11.*l15.*1.0e+1+A_init.*l9.*l10.*l15.*5.0-A_wayp.*l2.*l9.*l13.*1.2e+1+A_wayp.*l8.*l11.*l15.*1.0e+1-A_wayp.*l9.*l10.*l15.*5.0-A_init.*A_wayp.*T.*V_init.*l9.*2.4e+1-A_init.*A_wayp.*T.*V_wayp.*l9.*2.4e+1+A_init.*J_min.*P_init.*T.*l11.*4.8e+1+A_init.*J_max.*P_init.*T.*l9.*4.8e+1-A_init.*J_min.*P_wayp.*T.*l11.*4.8e+1-A_init.*J_max.*P_wayp.*T.*l9.*4.8e+1-A_wayp.*J_min.*P_init.*T.*l11.*4.8e+1-A_wayp.*J_max.*P_init.*T.*l9.*4.8e+1+A_wayp.*J_min.*P_wayp.*T.*l11.*4.8e+1+A_wayp.*J_max.*P_wayp.*T.*l9.*4.8e+1+A_init.*J_min.*V_init.*V_wayp.*l10.*2.4e+1-A_init.*J_max.*V_init.*V_wayp.*l8.*4.8e+1-A_wayp.*J_min.*V_init.*V_wayp.*l10.*2.4e+1+A_wayp.*J_max.*V_init.*V_wayp.*l8.*4.8e+1-A_init.*A_wayp.*J_max.*l9.*l14.*2.0e+1+J_min.*T.*V_init.*V_wayp.*l11.*2.4e+1+J_max.*T.*V_init.*V_wayp.*l9.*2.4e+1+A_init.*J_min.*V_init.*l11.*l13.*1.2e+1+A_init.*J_max.*V_init.*l9.*l13.*1.2e+1+A_init.*J_min.*V_wayp.*l11.*l13.*3.6e+1+A_init.*J_max.*V_wayp.*l9.*l13.*3.6e+1-A_wayp.*J_min.*V_init.*l11.*l13.*3.6e+1-A_wayp.*J_max.*V_init.*l9.*l13.*3.6e+1-A_wayp.*J_min.*V_wayp.*l11.*l13.*1.2e+1-A_wayp.*J_max.*V_wayp.*l9.*l13.*1.2e+1-A_init.*P_init.*T.*l8.*l10.*9.6e+1+A_init.*P_wayp.*T.*l8.*l10.*9.6e+1+A_wayp.*P_init.*T.*l8.*l10.*9.6e+1-A_wayp.*P_wayp.*T.*l8.*l10.*9.6e+1-J_min.*J_max.*T.*l2.*l5.*3.0e+1+J_min.*T.*V_init.*l5.*l10.*2.4e+1-J_max.*T.*V_init.*l5.*l8.*4.8e+1+J_min.*T.*V_wayp.*l2.*l10.*2.4e+1-J_max.*T.*V_wayp.*l2.*l8.*4.8e+1+A_init.*A_wayp.*l8.*l10.*l14.*4.0e+1-A_init.*J_min.*l5.*l10.*l13.*1.8e+1-A_init.*J_max.*l5.*l8.*l13.*2.4e+1+A_wayp.*J_min.*l2.*l10.*l13.*1.8e+1+A_wayp.*J_max.*l2.*l8.*l13.*2.4e+1-T.*V_init.*V_wayp.*l8.*l10.*4.8e+1-A_init.*V_init.*l8.*l10.*l13.*2.4e+1-A_init.*V_wayp.*l8.*l10.*l13.*7.2e+1+A_wayp.*V_init.*l8.*l10.*l13.*7.2e+1+A_wayp.*V_wayp.*l8.*l10.*l13.*2.4e+1-A_init.*A_wayp.*J_min.*P_init.*l10.*4.8e+1+A_init.*A_wayp.*J_max.*P_init.*l8.*9.6e+1+A_init.*A_wayp.*J_min.*P_wayp.*l10.*4.8e+1-A_init.*A_wayp.*J_max.*P_wayp.*l8.*9.6e+1+A_init.*J_min.*J_max.*T.*l6.*2.0e+1+A_wayp.*J_min.*J_max.*T.*l3.*2.0e+1-A_init.*A_wayp.*J_min.*T.*V_init.*l10.*2.4e+1+A_init.*A_wayp.*J_max.*T.*V_init.*l8.*4.8e+1-A_init.*A_wayp.*J_min.*T.*V_wayp.*l10.*2.4e+1+A_init.*A_wayp.*J_max.*T.*V_wayp.*l8.*4.8e+1];
 
t4 = roots(coefficients);
l2 = A_init.^2;
l3 = J_min.^2;
l4 = J_max.^2;
t3 = -(J_min.*l2.*2.0-J_max.*l2+V_init.*l4.*2.0-V_wayp.*l4.*2.0+A_wayp.^2.*J_max-A_init.*A_wayp.*J_min.*2.0-J_min.*J_max.*V_init.*2.0+J_min.*J_max.*V_wayp.*2.0+J_min.*T.^2.*l4+J_min.*l4.*t4.^2+A_init.*J_min.*J_max.*T.*2.0-A_wayp.*J_min.*J_max.*T.*2.0-A_init.*J_min.*J_max.*t4.*2.0+A_wayp.*J_min.*J_max.*t4.*2.0-J_min.*T.*l4.*t4.*2.0)./(A_init.*l3.*2.0-A_wayp.*l3.*2.0-A_init.*J_min.*J_max.*2.0+A_wayp.*J_min.*J_max.*2.0-J_min.*T.*l4.*2.0+J_max.*T.*l3.*2.0+J_min.*l4.*t4.*2.0-J_max.*l3.*t4.*2.0);

l2 = 1.0./J_min;
t7 = -(A_init-J_max.*(t3+t4+A_wayp.*l2)+J_min.*t3+J_max.*T)./(J_max.*(J_max.*l2-1.0));

t5 = (A_wayp-J_max.*t7)./J_min;

t1 = -(A_init+J_min.*t3)./J_max;

t6 = [0.0;0.0;0.0;0.0;0.0];

t2 = [0.0;0.0;0.0;0.0;0.0];

t = [t1, t2, t3, t4, t5, t6, t7];

end
