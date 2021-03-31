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

function [t_out,J_out,solution_out] = solve_O(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min) %#codegen
    
    num_valid = 0;
    num_times = 7;

    num_valid_max = 100;
    t_out = zeros(num_valid_max,num_times);
    J_out = zeros(num_valid_max,num_times);
    solution_out = int32(zeros(num_valid_max,1));
    
    n     = select_type(P_wayp,V_wayp,A_wayp);
    cond  = select_conditions(V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min);
    cases = select_cases_O(n,cond);
    
    
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
            case 0000
                t_test = zero_O(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0101
                t_test = abcdefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0102
                t_test = abcdefg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0103
                t_test = abcdeg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0104
                t_test = abcdeg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0105
                t_test = abcefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0106
                t_test = abcefg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0107
                t_test = abceg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0108
                t_test = abceg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0109
                t_test = acdefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0110
                t_test = acdefg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0111
                t_test = acdeg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0112
                t_test = acdeg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0113
                t_test = acefg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0114
                t_test = acefg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0115
                t_test = aceg_O_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0116
                t_test = aceg_O_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 0201
                t_test = abc_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0202
                t_test = abc_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0203
                t_test = abcde_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0204
                t_test = abcde_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0205
                t_test = abcdef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0206
                t_test = abcdef_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0207
                t_test = abcdefg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0208
                t_test = abcdefg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0209
                t_test = abcdeg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0210
                t_test = abcdeg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0211
                t_test = abcef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0212
                t_test = abcef_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0213
                t_test = abcefg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0214
                t_test = abcefg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0215
                t_test = abceg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0216
                t_test = abceg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0217
                t_test = ac_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0218
                t_test = ac_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0219
                t_test = acde_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0220
                t_test = acde_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0221
                t_test = acdef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0222
                t_test = acdef_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0223
                t_test = acdefg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0224
                t_test = acdefg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0225
                t_test = acdeg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0226
                t_test = acdeg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0227
                t_test = acef_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0228
                t_test = acef_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0229
                t_test = acefg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0230
                t_test = acefg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0231
                t_test = aceg_O_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0232
                t_test = aceg_O_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 0301
                t_test = abc_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0302
                t_test = abc_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0303
                t_test = abcde_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0304
                t_test = abcde_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0305
                t_test = abcdefg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0306
                t_test = abcdefg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0307
                t_test = abcefg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0308
                t_test = abcefg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0309
                t_test = abceg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0310
                t_test = abceg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0311
                t_test = ac_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0312
                t_test = ac_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0313
                t_test = acde_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0314
                t_test = acde_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0315
                t_test = acdefg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0316
                t_test = acdefg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0317
                t_test = acefg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0318
                t_test = acefg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0319
                t_test = aceg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0320
                t_test = aceg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0321
                t_test = abcdeg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0322
                t_test = abcdeg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0323
                t_test = acdeg_O_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0324
                t_test = acdeg_O_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 0401
                t_test = abc_O_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0402
                t_test = abc_O_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0403
                t_test = ac_O_AV(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0404
                t_test = ac_O_AV(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0501
                t_test = a_O_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0502
                t_test = a_O_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0503
                t_test = abc_O_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0504
                t_test = abc_O_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0505
                t_test = ac_O_A(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0506
                t_test = ac_O_A(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 0601
                t_test = a_O_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0602
                t_test = a_O_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0603
                t_test = ab_O_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0604
                t_test = ab_O_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0605
                t_test = abc_O_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0606
                t_test = abc_O_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0607
                t_test = ac_O_V(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0608
                t_test = ac_O_V(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 0701
                t_test = a_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0702
                t_test = a_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0703
                t_test = ab_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0704
                t_test = ab_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0705
                t_test = abc_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0706
                t_test = abc_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0707
                t_test = abcd_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0708
                t_test = abcd_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0709
                t_test = ac_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0710
                t_test = ac_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 0711
                t_test = acd_O_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 0712
                t_test = acd_O_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 01101
                t_test = abcdefg_NO_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01102
                t_test = abcdefg_NO_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01103
                t_test = abcdeg_NO_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01104
                t_test = abcdeg_NO_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01105
                t_test = acdefg_NO_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01106
                t_test = acdefg_NO_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01107
                t_test = acdeg_NO_AVP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01108
                t_test = acdeg_NO_AVP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 01201
                t_test = abcde_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01202
                t_test = abcde_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01203
                t_test = abcdef_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01204
                t_test = abcdef_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01205
                t_test = abcdefg_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01206
                t_test = abcdefg_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01207
                t_test = abcdeg_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01208
                t_test = abcdeg_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01209
                t_test = acde_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01210
                t_test = acde_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01211
                t_test = acdef_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01212
                t_test = acdef_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01213
                t_test = acdefg_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01214
                t_test = acdefg_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01215
                t_test = acdeg_NO_VP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01216
                t_test = acdeg_NO_VP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 01301
                t_test = abcde_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01302
                t_test = abcde_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01303
                t_test = acde_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01304
                t_test = acde_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01305
                t_test = abcdefg_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01306
                t_test = abcdefg_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01307
                t_test = acdefg_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01308
                t_test = acdefg_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01309
                t_test = abcdeg_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01310
                t_test = abcdeg_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01311
                t_test = acdeg_NO_AP(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01312
                t_test = acdeg_NO_AP(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
%-----------------------
            case 01701
                t_test = abcd_NO_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01702
                t_test = abcd_NO_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            case 01703
                t_test = acd_NO_P(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
            case 01704
                t_test = acd_NO_P(-P_init,-V_init,-A_init,-P_wayp,-V_wayp,-A_wayp,-V_min,-V_max,-A_min,-A_max,-J_min,-J_max);
            otherwise
                t_test = zero_O(P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
                fprintf('Error: Solution prior ');printint(solution);fprintf(' is not valid!\n');
        end
        valid = check(t_test,J_test,P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min);
        for index_t = 1:size(valid,2)
            if (valid(index_t) == true)
                t_test_valid = max(real(t_test(index_t,:)),0);
                b_existing = false;
                for idx = 1:num_valid
                    [t_test_simpl,J_test_simpl] = simplify_setp(t_test_valid,J_test);
                    [t_out_simpl,J_out_simpl] = simplify_setp(t_out(idx,:),J_out(idx,:));
                    t_tol = 0.0000001; %numerical
                    J_tol = 0.0000001; %numerical
                    b_existing = (size(t_test_simpl,2) == size(t_out_simpl,2) && size(J_test_simpl,2) == size(J_out_simpl,2)) && all(abs(t_test_simpl - t_out_simpl) < t_tol) && all(abs(J_test_simpl - J_out_simpl) < J_tol);
                    if (b_existing == true)
                        break;
                    end
                end
                if (b_existing == false)
                    num_valid = num_valid + 1;
                    t_out(num_valid,:) = t_test_valid;
                    J_out(num_valid,:) = J_test;
                    solution_out(num_valid,:) = solution;
                end
            end
        end
    end
    t_out = t_out(1:num_valid,:);
    J_out = J_out(1:num_valid,:);
    solution_out = solution_out(1:num_valid,:);

    if (num_valid == 0)
        fprintf('Error: Could not find valid optimal solution!\n');
    elseif (num_valid > 1)
        fprintf('Debug: Multiple (');printint(num_valid);fprintf(') optimal solutions!\n');
        [~,index_sorted] = sort(sum(t_out,2));
        t_out = t_out(index_sorted,:);
        J_out = J_out(index_sorted,:);
        solution_out = solution_out(index_sorted,:);
    end
end
    