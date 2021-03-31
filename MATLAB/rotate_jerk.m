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

function [t_out_1,J_out_1,t_out_2,J_out_2] = rotate_jerk(alpha,t_1,J_1,t_2,J_2) %#codegen

    t_cumsum_x = cumsum(t_1(1,:));
    t_cumsum_y = cumsum(t_2(1,:));
    t_cumsum = unique([t_cumsum_x,t_cumsum_y]);

    J_rot = zeros(2,size(t_cumsum,2));

    J_x = horzcat(J_1,0); % Assume zero Jerk input after trajectory
    J_y = horzcat(J_2,0); % Assume zero Jerk input after trajectory

    for index = 1:size(t_cumsum,2)

        index_x = find(t_cumsum_x>=t_cumsum(index),1,'first');
        index_y = find(t_cumsum_y>=t_cumsum(index),1,'first');

        if(isempty(index_x))
            index_x = size(t_1,2)+1; % Assume zero Jerk input after trajectory
        end

        if(isempty(index_y))
            index_y = size(t_2,2)+1; % Assume zero Jerk input after trajectory
        end

        J_rot(1:2,index) = rotate_state(-alpha,[J_x(index_x);J_y(index_y)]);

    end

    t_out_1 = diff(cat(2,0,t_cumsum));
    t_out_2 = diff(cat(2,0,t_cumsum));
    J_out_1 = J_rot(1,:);
    J_out_2 = J_rot(2,:);

end