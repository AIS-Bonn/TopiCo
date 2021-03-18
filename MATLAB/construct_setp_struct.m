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

function [J_setp_struct] = construct_setp_struct(t_in2,J_in2) %#codegen

    if (~iscell(t_in2))
        t_in = {t_in2};
    else
        t_in = t_in2;
    end
    
    if (~iscell(J_in2))
        J_in = {J_in2};
    else
        J_in = J_in2;
    end
    
    simplify = true;
    num_axes = size(t_in,1);
    num_waypoints = size(t_in,2);
     
    s.time = 0;
    s.signals.values = 0;
    J_setp_struct = repmat(s,1,num_axes);
    coder.varsize('J_setp_struct(:).time',[],[true, true]);
    coder.varsize('J_setp_struct(:).signals.values',[],[true, true]);

    for index_axis = 1:num_axes

        t = zeros(1,0);
        J = zeros(1,0);
        for index_waypoint = 1:num_waypoints
            t = [t,t_in{index_axis,index_waypoint}];
            J = [J,J_in{index_axis,index_waypoint}];
        end
        if (simplify == true)
            [t_2,J_2] = simplify_setp(t(1,:),J(1,:));
        else
            t_2 = t;
            J_2 = J;
        end

        J_setp_struct(index_axis).time = cat(2,0,cumsum(t_2))';
        J_setp_struct(index_axis).signals.values = cat(2,J_2,0);
    
    end

end