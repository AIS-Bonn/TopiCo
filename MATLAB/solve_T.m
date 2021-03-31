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

function [t_out,J_out,solution_out] = solve_T(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync,b_sync_V,b_sync_A,b_sync_J,b_sync_W,direction) %#codegen
    
    num_valid = 0;
    num_times = 11;
     
    num_valid_max = 100;
    t_out = zeros(num_valid_max,num_times);
    J_out = zeros(num_valid_max,num_times);
    solution_out = int32(zeros(num_valid_max,1));
    
    coder.varsize('t_out',[], [true false]);
    coder.varsize('J_out',[], [true false]);
    coder.varsize('solution_out',[], [true false]);

    
    if (b_sync_W == true && ~isnan(P_wayp) && ~isnan(V_wayp) && ~isnan(A_wayp))
        [t_1,J_1,~] = solve_O(P_init,V_init,A_init,NaN,0,0,V_max,V_min,A_max,A_min,J_max,J_min);
        [P_int,V_int,A_int] = evaluate_to_time(P_init,V_init,A_init,construct_setp_struct(t_1,J_1));
        [t_2,J_2,~] = solve_O(P_int,V_int,A_int,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
        if (sum(t_1(1:3)) + sum(t_2(1:7)) < t_sync)
            num_valid = num_valid + 1;
            t_out(num_valid,1:3) = t_1(1:3);
            J_out(num_valid,1:3) = J_1(1:3);
            t_out(num_valid,5:11) = t_2(1:7);
            J_out(num_valid,5:11) = J_2(1:7);
            t_out(num_valid,4) = t_sync - sum(t_1(1:3)) - sum(t_2(1:7));
            J_out(num_valid,4) = 0;
            solution_out = int32(11111);
            return;
        end
    end
    
    
    cases = int32([]);
    if (~isnan(P_wayp) && ~isnan(V_wayp) && ~isnan(A_wayp))
        n = select_type(P_wayp,V_wayp,A_wayp);
        cond = select_conditions(V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min);
        cases_TV = select_cases_TV(n,cond,b_sync_V);
        cases_TA = select_cases_TA(n,cond,b_sync_A);
        cases_TJ = select_cases_TJ(n,cond,b_sync_J);
        cases = [cases_TV, cases_TA, cases_TJ];  
    elseif (~isnan(P_wayp) && ~isnan(V_wayp) && isnan(A_wayp))  
        if (direction == 1)
            cases = int32([40202]);
        elseif (direction == -1)
            cases = int32([40201]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end
    elseif (~isnan(P_wayp) && isnan(V_wayp) && ~isnan(A_wayp))
        if (direction == 1)
            cases = int32([40302, 40304, 40306, 40308, 40310, 40312, 40314, 40316]);
        elseif (direction == -1)
            cases = int32([40301, 40303, 40305, 40307, 40309, 40311, 40313, 40315]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end
    elseif (~isnan(P_wayp) && isnan(V_wayp) && isnan(A_wayp))
        if (direction == 1)
            cases = int32([40702, 40704, 40706, 40708, 40710, 40712, 40714, 40716, 40718, 40720, 40722, 40724, 40726, 40728, 40730, 40732]);
        elseif (direction == -1)
            cases = int32([40701, 40703, 40705, 40707, 40709, 40711, 40713, 40715, 40717, 40719, 40721, 40723, 40725, 40727, 40729, 40731]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end
    elseif (isnan(P_wayp) && ~isnan(V_wayp) && ~isnan(A_wayp))
        if (direction == 1)
            cases = int32([40402, 40404, 40406, 40408, 40410, 40412, 40414, 40416]);
        elseif (direction == -1)
            cases = int32([40401, 40403, 40405, 40407, 40409, 40411, 40413, 40415]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end
    elseif (isnan(P_wayp) && ~isnan(V_wayp) && isnan(A_wayp))
        if (direction == 1)
            cases = int32([40602, 40604, 40606, 40608, 40610, 40612, 40614, 40616, 40618, 40620, 40622, 40624, 40626, 40628, 40630, 40632]);
        elseif (direction == -1)
            cases = int32([40601, 40603, 40605, 40607, 40609, 40611, 40613, 40615, 40617, 40619, 40621, 40623, 40625, 40627, 40629, 40631]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end
    elseif (isnan(P_wayp) && isnan(V_wayp) && ~isnan(A_wayp))
        if (direction == 1)
            cases = int32([40502, 40504, 40506, 40508, 40510, 40512, 40514, 40516, 40518, 40520, 40522, 40524]);
        elseif (direction == -1)
            cases = int32([40501, 40503, 40505, 40507, 40509, 40511, 40513, 40515, 40517, 40519, 40521, 40523]);
        else
            fprintf('Error: Direction not -1 or 1 and free DoF!\n');
        end 
    else
        fprintf('Error: P_wayp and V_wayp and A_wayp = NaN!\n');
    end
    
    
    cases = [cases, 00000, 21109, 21110]; %TODO Put these into appropriate places. They are sometimes necessary to find a solution!

    num_cases = size(cases,2);
    for index = 1:num_cases
        solution = cases(index);
        if (mod(floor(double(solution)/1000),2) == 0)
            if (mod(solution,2) == 1)
                J_test = [J_max, 0.0, J_min, 0.0, J_min, 0.0, J_max];
            else
                J_test = [J_min, 0.0, J_max, 0.0, J_max, 0.0, J_min];
            end
        else
            if (mod(solution,2) == 1)
                J_test = [J_min, 0.0, J_max, 0.0, J_min, 0.0, J_max];
            else
                J_test = [J_max, 0.0, J_min, 0.0, J_max, 0.0, J_min];
            end
        end

        switch solution
            case 00000
                t_test = zero_O(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
                t_test(4) = t_sync;
            case 10101
                t_test = abcdefg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 10102
                t_test = abcdefg_TV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 10103
                t_test = abcdeg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 10104
                t_test = abcdeg_TV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 10105
                t_test = acdefg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 10106
                t_test = acdefg_TV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 10107
                t_test = acdeg_TV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 10108
                t_test = acdeg_TV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 20101
                t_test = abcdefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20102
                t_test = abcdefg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 20103
                t_test = abcdeg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20104   
                t_test = abcdeg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 20105
                t_test = abcefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20106
                t_test = abcefg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 20107
                t_test = abceg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20108
                t_test = abceg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 20109
                t_test = acdefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20110
                t_test = acdefg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 20111
                t_test = acefg_TA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 20112
                t_test = acefg_TA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 30101
                t_test = abcdefg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30102
                t_test = abcdefg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30103
                t_test = abcdeg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30104
                t_test = abcdeg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30105
                t_test = abcefg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30106
                t_test = abcefg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30107
                t_test = abceg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30108
                t_test = abceg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30109
                t_test = acdefg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30110
                t_test = acdefg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30111
                t_test = acdeg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30112
                t_test = acdeg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30113
                t_test = acefg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30114
                t_test = acefg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 30115
                t_test = aceg_TJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 30116
                t_test = aceg_TJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync); 
%-----------------------
            case 11101
                t_test = abcdefg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 11102
                t_test = abcdefg_NTV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 11103
                t_test = abcdeg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 11104
                t_test = abcdeg_NTV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 11105
                t_test = acdefg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 11106
                t_test = acdefg_NTV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 11107
                t_test = acdeg_NTV_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 11108
                t_test = acdeg_NTV_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 21101
                t_test = abcdefg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21102
                t_test = abcdefg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 21103
                t_test = abcdeg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21104
                t_test = abcdeg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 21105
                t_test = abcefg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21106
                t_test = abcefg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 21107
                t_test = abceg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21108
                t_test = abceg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 21109
                t_test = acdefg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21110
                t_test = acdefg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 21111
                t_test = acefg_NTA_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 21112
                t_test = acefg_NTA_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 31101
                t_test = abcdefg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31102
                t_test = abcdefg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31103
                t_test = abcdeg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31104
                t_test = abcdeg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31105
                t_test = abcefg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31106
                t_test = abcefg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31107
                t_test = abceg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31108
                t_test = abceg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31109
                t_test = acdefg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31110
                t_test = acdefg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31111
                t_test = acdeg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31112
                t_test = acdeg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31113
                t_test = acefg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31114
                t_test = acefg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 31115
                t_test = aceg_NTJ_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 31116
                t_test = aceg_NTJ_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 40201
                t_test = abcdefg_T_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40202
                t_test = abcdefg_T_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 40301
                t_test = abcdefg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40302
                t_test = abcdefg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40303
                t_test = abcefg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40304
                t_test = abcefg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40305
                t_test = acefg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40306
                t_test = acefg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40307
                t_test = abceg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40308
                t_test = abceg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40309
                t_test = aceg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40310
                t_test = aceg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40311
                t_test = acdefg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40312
                t_test = acdefg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40313
                t_test = abcdeg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40314
                t_test = abcdeg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40315
                t_test = acdeg_T_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40316
                t_test = acdeg_T_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------              
            case 40401
                t_test = aceg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40402
                t_test = aceg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40403
                t_test = abceg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40404
                t_test = abceg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40405
                t_test = acefg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40406
                t_test = acefg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40407
                t_test = abcefg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40408
                t_test = abcefg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40409
                t_test = abcdefg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40410
                t_test = abcdefg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40411
                t_test = acdefg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40412
                t_test = acdefg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40413
                t_test = acdeg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40414
                t_test = acdeg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40415
                t_test = abcdeg_T_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40416
                t_test = abcdeg_T_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 40501
                t_test = ac_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40502
                t_test = ac_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40503
                t_test = abc_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40504
                t_test = abc_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40505
                t_test = abcde_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40506
                t_test = abcde_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40507
                t_test = acde_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40508
                t_test = acde_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40509
                t_test = abcdeg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40510
                t_test = abcdeg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40511
                t_test = abcdefg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40512
                t_test = abcdefg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40513
                t_test = acdefg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40514
                t_test = acdefg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40515
                t_test = acdeg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40516
                t_test = acdeg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40517
                t_test = aceg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40518
                t_test = aceg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40519
                t_test = acefg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40520
                t_test = acefg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40521
                t_test = abceg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40522
                t_test = abceg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40523
                t_test = abcefg_T_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40524
                t_test = abcefg_T_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 40601
                t_test = ac_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40602
                t_test = ac_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40603
                t_test = abc_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40604
                t_test = abc_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40605
                t_test = abcef_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40606
                t_test = abcef_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40607
                t_test = acef_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40608
                t_test = acef_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40609
                t_test = aceg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40610
                t_test = aceg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40611
                t_test = abceg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40612
                t_test = abceg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40613
                t_test = abcefg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40614
                t_test = abcefg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40615
                t_test = abcdefg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40616
                t_test = abcdefg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40617
                t_test = abcdef_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40618
                t_test = abcdef_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40619
                t_test = acdef_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40620
                t_test = acdef_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40621
                t_test = acdefg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40622
                t_test = acdefg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40623
                t_test = acdeg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40624
                t_test = acdeg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40625
                t_test = abcdeg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40626
                t_test = abcdeg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40627
                t_test = acefg_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40628
                t_test = acefg_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40629
                t_test = abcde_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40630
                t_test = abcde_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40631
                t_test = acde_T_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40632
                t_test = acde_T_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
%-----------------------
            case 40701
                t_test = abcef_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40702
                t_test = abcef_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40703
                t_test = acef_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40704
                t_test = acef_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40705
                t_test = ac_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40706
                t_test = ac_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40707
                t_test = abcdef_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40708
                t_test = abcdef_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40709
                t_test = acdef_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40710
                t_test = acdef_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40711
                t_test = abc_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40712
                t_test = abc_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40713
                t_test = abcdefg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40714
                t_test = abcdefg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40715
                t_test = abcdeg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40716
                t_test = abcdeg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40717
                t_test = aceg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40718
                t_test = aceg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40719
                t_test = abceg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40720
                t_test = abceg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40721
                t_test = abcefg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40722
                t_test = abcefg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40723
                t_test = acefg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40724
                t_test = acefg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40725
                t_test = abcde_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40726
                t_test = abcde_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40727
                t_test = acde_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40728
                t_test = acde_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40729
                t_test = acdeg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40730
                t_test = acdeg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            case 40731
                t_test = acdefg_T_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
            case 40732
                t_test = acdefg_T_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max,t_sync);
            otherwise
                t_test = zero_O(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
                fprintf('Error: Solution prior ');printint(solution);fprintf(' is not valid!\n');
        end
        valid = check(t_test,J_test,P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync);
        for index_t = 1:size(valid,2)
            if (valid(index_t) == true)
                t_test_valid = max(real(t_test(index_t,:)),0);
                b_existing = false;
                for idx = 1:num_valid
                    [t_test_simpl,J_test_simpl] = simplify_setp(t_test_valid,J_test);
                    [t_out_simpl,J_out_simpl] = simplify_setp(t_out(idx,1:7),J_out(idx,1:7));
                    t_tol = 0.0000001; %numerical
                    J_tol = 0.0000001; %numerical
                    b_existing = (size(t_test_simpl,2) == size(t_out_simpl,2) && size(J_test_simpl,2) == size(J_out_simpl,2)) && all(abs(t_test_simpl - t_out_simpl) < t_tol) && all(abs(J_test_simpl - J_out_simpl) < J_tol);
                    if (b_existing == true)
                        break;
                    end
                end
                if (b_existing == false)
                    num_valid = num_valid + 1;
                    t_out(num_valid,1:7) = t_test_valid;
                    J_out(num_valid,1:7) = J_test;
                    t_out(num_valid,8:11) = 0;
                    J_out(num_valid,8:11) = 0;
                    solution_out(num_valid,:) = solution;
                end
            end
        end
    end
   
    t_out = t_out(1:num_valid,:);
    J_out = J_out(1:num_valid,:);
    solution_out = solution_out(1:num_valid,:);
    
    if (num_valid > 0) %So that sync time is really the same. adjust t4 so that J == 0 and induced error is minimal
        t_out(:,4) = t_out(:,4) - sum(t_out,2) + t_sync; 
    end
    
    if (num_valid == 0)
        fprintf('Error: Could not find valid timed solution!\n');
    elseif (num_valid > 1)
        fprintf('Debug: Multiple (');printint(num_valid);fprintf(') timed solutions!\n');
        [~,index_sorted] = sort(sum(solution_out,2),'descend'); %TODO Here should be a better sorting instead of sorting via the Case ID
        t_out = t_out(index_sorted,:);
        J_out = J_out(index_sorted,:);
        solution_out = solution_out(index_sorted,:);
    end
end
