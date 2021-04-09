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

function [valid] = check(t,J,P_init,V_init,A_init,P_wayp,V_wayp,A_wayp,V_max,V_min,A_max,A_min,J_max,J_min,t_sync) %#codegen
    
    P_tol      =  0.0001;         %numerical
    V_tol      =  0.0001;         %numerical
    A_tol      =  0.0001;         %numerical
    t_tol      =  0.001;          %numerical

    V_max      =  1.0001 * V_max; %numerical
    V_min      =  1.0001 * V_min; %numerical
    A_max      =  1.0001 * A_max; %numerical
    A_min      =  1.0001 * A_min; %numerical
    J_max      =  1.0001 * J_max; %numerical
    J_min      =  1.0001 * J_min; %numerical
    t_min      = -0.0001;         %numerical
    t_max_imag =  0.0001;         %numerical
    

    
    num_t = size(t,1);
    valid = false(1,num_t);
    

    
    for index = 1:num_t
        if (all(imag(t(index,:)) <= t_max_imag) && (all(t(index,:) >= t_min)))
            sum_t = sum(t(index,:));
            if (nargin < 15 || t_sync - t_tol <  sum_t && t_sync + t_tol > sum_t)
                [p,v,a] = evaluate(real(t(index,:)),J,P_init,V_init,A_init);

                valid_0 = check_feasibility(V_init,A_init,V_max,V_min,A_max,A_min,J_max,J_min);
                valid_2 = check_feasibility(v(2),a(2),V_max,V_min,A_max,A_min,J_max,J_min);
                valid_7 = check_feasibility(v(7),a(7),V_max,V_min,A_max,A_min,J_max,J_min);
               
                if ((all(v <= V_max) && all(v >= V_min) && valid_0 == 0 && valid_2 == 0 && valid_7 == 0 || ...
                    all(v(3:end) <= V_max) && all(v(3:end) >= V_min) && valid_0 ~= 0 || ...
                    all(v <= V_max) && all(v >= V_min) && ~isnan(V_wayp) && ~isnan(A_wayp) && (valid_0 == 0 || valid_0 == 5) && (valid_2 == 0 || valid_2 == 5) && valid_7 == 5 && ...
                    all(v <= V_max) && all(v >= V_min) && ~isnan(V_wayp) && ~isnan(A_wayp) && (valid_0 == 0 || valid_0 == 6) && (valid_2 == 0 || valid_2 == 6) && valid_7 == 6) && ...   
                    all(a <= A_max) && all(a >= A_min) && ...
                    all(J <= J_max) && all(J >= J_min) && ...
                    (p(end) <= P_wayp + P_tol && p(end) >= P_wayp - P_tol || isnan(P_wayp)) && ...
                    (v(end) <= V_wayp + V_tol && v(end) >= V_wayp - V_tol || isnan(V_wayp)) && ...
                    (a(end) <= A_wayp + A_tol && a(end) >= A_wayp - A_tol || isnan(A_wayp)))
                        valid(index) = true;
                end
            end
        end   
    end 
end





        
