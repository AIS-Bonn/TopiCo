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

if (~exist('resolution','var'))
    resolution        = [-2880, 1620, 1920, 1080];
end
if (~exist('viewangle','var'))
    viewangle         = [-25,18];
end
if (~exist('t_video_start','var'))
    t_video_start     = 0.0;
end
if (~exist('t_video_stop','var'))
    t_video_stop      = Inf;
end
if (~exist('size_font','var'))
    size_font         = 18;
end
if (~exist('tickformat','var'))
    tickformat        = {'%.1f','%.1f','%.1f','%.1f'};
end
if (~exist('ticks','var'))
    ticks             = [1.0;1.0;1.0];
end
if (~exist('size_deriv','var'))
    size_deriv        = 50;
end
if (~exist('size_path','var'))
    size_path         = 3.0;
end
if (~exist('t_draw_MAV','var'))
    t_draw_MAV        = 0.0;
end
if (~exist('index_MAV_bb','var'))
    index_MAV_bb      = uint8(1);
end
if (~exist('size_MAV_line','var'))
    size_MAV_line     = 0.03;
end
if (~exist('size_MAV_line_bb','var'))
    size_MAV_line_bb  = 0.015;
end
if (~exist('size_MAV_sphere','var'))
    size_MAV_sphere   = 0.08;
end
if (~exist('size_traj','var'))
    size_traj         = 3.0;
end
if (~exist('size_point','var'))
    size_point        = 300;
end
if (~exist('size_sphere','var'))
    size_sphere       = 0.08;
end
if (~exist('size_ring','var'))
    size_ring         = 2.0;
end
if (~exist('fps','var'))
    fps               = 30;
end
if (~exist('b_clf','var'))
    b_clf             = true;
end
if (~exist('viewspeed','var'))
    viewspeed         = [0.0, 0.0];
end
if (~exist('g','var'))
    g                 = 9.81;
end
if (~exist('index_deriv','var'))
    index_deriv       = uint8(1);
end
if (~exist('index_infl_axes','var'))
    index_infl_axes   = uint8(2);
end
if (~exist('color_startpoint','var'))
    color_startpoint  = [0.0, 0.7, 0.0];
end
if (~exist('color_waypoint','var'))
    color_waypoint    = [0.0, 0.0, 0.7];
end
if (~exist('color_endpoint','var'))
    color_endpoint    = [0.7, 0.0, 0.0];
end
if (~exist('b_rotate_velocity','var'))
    b_rotate_velocity = false;
end
if (~exist('b_draw_text','var'))
    b_draw_text       = false(5,1);
end
if (~exist('angle_factor','var'))
    angle_factor      = 0.8;
end
if (~exist('angle_max_vel','var'))
    angle_max_vel     = 15;
end
if (~exist('angle_min_vel','var'))
    angle_min_vel     = -15;
end
if (~exist('alpha_MAV','var'))
    alpha_MAV         = 1.0;
end
if (~exist('alpha_MAV_traj','var'))
    alpha_MAV_traj    = 1.0;
end
if (~exist('alpha_MAV_buff','var'))
    alpha_MAV_buff    = 0.0;
end
if (~exist('alpha_deriv','var'))
    alpha_deriv       = 1.0;
end
if (~exist('alpha_deriv_buff','var'))
    alpha_deriv_buff  = 0.1;
end
if (~exist('size_buff','var'))
    size_buff         = 1;
end
if (~exist('b_video_rollout','var'))
    b_video_rollout   = false;
end
if (~exist('b_record_video','var'))
    b_record_video    = false;
end
if (~exist('MAV_zoom','var'))
    MAV_zoom          = 5.0;
end
if (~exist('MAV_d','var'))
    MAV_d             = [0.5;0.5;0.2];
end
if (~exist('colorbar_margin','var'))
    colorbar_margin   = 0.1;
end
if (~exist('colorbar_width','var'))
    colorbar_width    = 0.02;
end
