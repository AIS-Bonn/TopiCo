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
if (~exist('size_font','var'))
    size_font         = 18;
end
if (~exist('tickformat','var'))
    tickformat        = {'%.1f','%.1f','%.1f','%.1f'};
end
if (~exist('size_marker','var'))
    size_marker       = 50;
end
if (~exist('size_traj','var'))
    size_traj         = 3.0;
end
if (~exist('b_clf','var'))
    b_clf             = true;
end
if (~exist('size_path','var'))
    size_path         = 3.0;
end
if (~exist('size_point','var'))
    size_point        = 150;
end
if (~exist('size_segment','var'))
    size_segment      = 1.0;
end
if (~exist('size_limit','var'))
    size_limit        = 1.5;
end
if (~exist('size_ring','var'))
    size_ring         = 2.0;
end
if (~exist('b_plot_markers','var'))
    b_plot_markers    = true(4,1);
end
if (~exist('b_plot_numbers','var'))
    b_plot_numbers    = false;
end
if (~exist('b_plot_path','var'))
    b_plot_path       = false;
end
if (~exist('b_record_video','var'))
    b_record_video    = false;
end
if (~exist('b_full_axes','var'))
    b_full_axes       = false(num_axes,1);
end
if (~exist('b_plot_axis','var'))
    b_plot_axis       = true(num_axes,1);
end
if (~exist('axislimits_1D','var'))
    axislimits_1D     = zeros(4,4);
end
if (~exist('axisnames','var'))
    axisnames = cell(num_axes,1);
    for index_axis = 1:num_axes
        axisnames{index_axis} = ['Axis ',int2str(index_axis)];
    end
end
if (~exist('color_line','var'))
    color_line        = lines(num_axes);
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
if (~exist('color_limit','var'))
    color_limit       = [1.0, 0.0, 0.0];
end
if (~exist('color_marker','var'))
    color_marker      = [0.0, 0.0, 0.0];
end
if (~exist('alpha_limit','var'))
    alpha_limit       = 0.05;
end
