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

function [t,J] = correct_feasibility(P_init,V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min,b_hard_V_lim) %#codegen

    num_times = 4;
    t = zeros(1,num_times);
    J = zeros(1,num_times);

    feasibility = check_feasibility(V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min);
    if (feasibility == 1)
        t_test = a_O_A(-P_init,-V_init,-A_init,NaN,NaN,-A_max,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
        t(1) = real(t_test(1));
        J(1) = J_min;
    elseif (feasibility == 2)
        t_test = a_O_A(P_init,V_init,A_init,NaN,NaN,A_min,V_max,V_min,A_max,A_min,J_max,J_min);
        t(1) = real(t_test(1));
        J(1) = J_max;
    end
    if (feasibility == 1 || feasibility == 2)
        [P_init,V_init,A_init] = evaluate_to_time(P_init,V_init,A_init,construct_setp_struct({t(1)},{J(1)}));
        feasibility = check_feasibility(V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min);
    end
    if (b_hard_V_lim == true && feasibility == 3)
        [t_test,J_test,~] = solve_O(P_init,V_init,A_init,NaN,V_max,NaN,Inf,V_min,A_max,A_min,J_max,J_min); %V_max =  Inf wegen check, V_min muss bleiben für Prädiktion und 3-phasiges t
        t(2:4) = t_test(1:3);
        J(2:4) = J_test(1:3);
    elseif (b_hard_V_lim == true && feasibility == 4)
        [t_test,J_test,~] = solve_O(P_init,V_init,A_init,NaN,V_min,NaN,V_max,-Inf,A_max,A_min,J_max,J_min);%V_min = -Inf wegen check, V_max muss bleiben für Prädiktion und 3-phasiges t
        t(2:4) = t_test(1:3);
        J(2:4) = J_test(1:3);
    elseif (b_hard_V_lim == true && feasibility == 5)
        t_test1 = a_O_A(-P_init,-V_init,-A_init,NaN,NaN,0.0,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
        t_test1 = real(t_test1);
        [P_init,V_init,A_init] = evaluate_to_time(P_init,V_init,A_init,construct_setp_struct({t_test1(1)},{J_min}));
        [t_test,J_test,~] = solve_O(P_init,V_init,A_init,NaN,V_max,NaN,Inf,V_min,A_max,A_min,J_max,J_min);%V_max =  Inf wegen check, V_min muss bleiben für Prädiktion und 3-phasiges t
        t(2:4) = t_test(1:3) + t_test1(1:3);
        J(2:4) = J_test(1:3);
    elseif (b_hard_V_lim == true && feasibility == 6)
        t_test1 = a_O_A(P_init,V_init,A_init,NaN,NaN,0.0,V_max,V_min,A_max,A_min,J_max,J_min);
        t_test1 = real(t_test1);
        [P_init,V_init,A_init] = evaluate_to_time(P_init,V_init,A_init,construct_setp_struct({t_test1(1)},{J_max}));
        [t_test,J_test,~] = solve_O(P_init,V_init,A_init,NaN,V_min,NaN,V_max,-Inf,A_max,A_min,J_max,J_min);%V_min = -Inf wegen check, V_max muss bleiben für Prädiktion und 3-phasiges t
        t(2:4) = t_test(1:3) + t_test1(1:3);
        J(2:4) = J_test(1:3);
    end

end