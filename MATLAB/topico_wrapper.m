% ---------------------------------------------------------------------
% Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
% Version:    2021-04-07 13:04:59
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

function [J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = topico_wrapper(State_start,Waypoints,V_max_in,V_min_in,A_max_in,A_min_in,J_max_in,J_min_in,A_global,b_sync_V_in,b_sync_A_in,b_sync_J_in,b_sync_W_in,b_rotate_in,b_hard_V_lim_in,b_catch_up_in,direction_in,ts_rollout) %#codegen
    % This wrapper only exist to prohibit namespace conflicts with the C++ ROS Node. Thus, we generate code with the "topico_wrapper" entry function, instead of just "topico".
    [J_setp_struct,solution_out,T_waypoints,P,V,A,J,t] = topico(State_start,Waypoints,V_max_in,V_min_in,A_max_in,A_min_in,J_max_in,J_min_in,A_global,b_sync_V_in,b_sync_A_in,b_sync_J_in,b_sync_W_in,b_rotate_in,b_hard_V_lim_in,b_catch_up_in,direction_in,ts_rollout);
    
end
