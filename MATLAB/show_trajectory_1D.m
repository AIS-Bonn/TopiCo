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

show_trajectory_1D_defaults;
if (~exist('h_figure1D','var') || ~isvalid(h_figure1D))
    h_figure1D = figure('Color','w','Units','pixels','Resize','off');
    for i = 1:1:10 %MATLAB Bug
        h_figure1D.Position = resolution;
        drawnow;
    end
else
    figure(h_figure1D);
end
if (b_clf == true)
    clf;
end

num_times = zeros(num_axes,1);
for index_axis= 1:num_axes
    num_times(index_axis) = size(J_setp_struct(index_axis).time,1);
end

h_stairs      = zeros(num_axes,4);
h_marker      = zeros(num_axes,4,max(num_times));
h_start       = zeros(num_axes,3);
h_waypoints_1 = gobjects(num_axes,3,num_waypoints);
h_waypoints_2 = gobjects(num_axes,3,num_waypoints);
h_limits_1    = zeros(num_axes,3,num_waypoints);
h_limits_2    = zeros(num_axes,3,num_waypoints);

index_stop = ones(num_axes,1)*size(t,2);
for index_axis = 1:num_axes
    if (b_full_axes(index_axis) == false)
        index_stop(index_axis,:) = find(t(index_axis,:) <= sum(T_waypoints(index_axis,:)),1,'last');
    end
end

axislimits_1D(:,2) = max(sum(T_waypoints,2));
for index_axis = 1:num_axes
    if (b_plot_axis(index_axis) == true)
        axislimits_1D(1,3) = min(axislimits_1D(1,3),min(P(index_axis,1:index_stop(index_axis))));
        axislimits_1D(1,4) = max(axislimits_1D(1,4),max(P(index_axis,1:index_stop(index_axis))));
        axislimits_1D(2,3) = min(min(axislimits_1D(2,3),min(V(index_axis,1:index_stop(index_axis)))),min(0));
        axislimits_1D(2,4) = max(max(axislimits_1D(2,4),max(V(index_axis,1:index_stop(index_axis)))),max(0));
        axislimits_1D(3,3) = min(min(axislimits_1D(3,3),min(A(index_axis,1:index_stop(index_axis)))),min(0));
        axislimits_1D(3,4) = max(max(axislimits_1D(3,4),max(A(index_axis,1:index_stop(index_axis)))),max(0));
        axislimits_1D(4,3) = min(min(axislimits_1D(4,3),min(J_setp_struct(index_axis).signals.values)),min(0));
        axislimits_1D(4,4) = max(max(axislimits_1D(4,4),max(J_setp_struct(index_axis).signals.values)),max(0));
    end
end
margin = (axislimits_1D(:,4)-axislimits_1D(:,3))/10;
axislimits_1D(:,3) = axislimits_1D(:,3) - margin;
axislimits_1D(:,4) = axislimits_1D(:,4) + margin;

if (~exist('h_axis1D','var') || size(h_axis1D,2) < 1 || ~isvalid(h_axis1D(1)))
    h_axis1D(1) = subplot(4,1,1);
else
    h_figure1D.CurrentAxes = h_axis1D(1);
end
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            h_stairs(index_axis,1) = stairs(t(index_axis,1:index_stop(index_axis)),P(index_axis,1:index_stop(index_axis)),'Color',color_line(index_axis,:),'LineWidth',size_traj);
            h_start(index_axis,1) = scatter(0,State_start(index_axis,1),size_point,color_startpoint,'Marker','o','MarkerFaceColor',color_startpoint);
            for index_waypoint = 1:1:num_waypoints
                if (index_waypoint == num_waypoints)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                line([T_old T],[0 0],'Color','black','LineStyle',':','Linewidth',size_segment);
                h_waypoints_1(index_axis,1,index_waypoint) = scatter(0,Waypoints(index_axis,1,index_waypoint),size_point,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                h_waypoints_2(index_axis,1,index_waypoint) = scatter(T,Waypoint_evolved(index_axis,1),size_point,color,'Marker','o','MarkerFaceColor',color);
                if (b_plot_path == true)
                    xt = @(t) Waypoints(index_axis,1,index_waypoint)+Waypoints(index_axis,4,index_waypoint).*t+1/2.*Waypoints(index_axis,5,index_waypoint).*t.^2;
                    fplot(xt,[0 T],'Color',color,'LineStyle',':','Linewidth',size_path);
                    scatter(0,Waypoints(index_axis,1,index_waypoint),size_point,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0,'LineWidth',size_ring);
                end
            end
            for index_times = 2:1:num_times(index_axis)
                t_marker = J_setp_struct(index_axis).time(index_times) - ts_rollout;
                line([t_marker, t_marker],[axislimits_1D(1,3), axislimits_1D(1,4)],'Color','black','LineStyle',':','Linewidth',size_segment);
                if (b_plot_numbers == true)
                    text(J_setp_struct(index_axis).time(index_times-1) + (J_setp_struct(index_axis).time(index_times) - J_setp_struct(index_axis).time(index_times-1)) / 2,axislimits_1D(1,4),int2str(index_times-1),'FontSize',size_font,'FontName','Times','HorizontalAlignment','center','VerticalAlignment','bottom');
                end
                if (b_plot_markers(1) == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),t_marker);
                    h_marker(index_axis,1,index_times) = scatter(t_marker,P_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
        end
    end
if (~exist('h_axis1D','var') || size(h_axis1D,2) < 2 || ~isvalid(h_axis1D(2)))
    h_axis1D(2) = subplot(4,1,2);
else
    h_figure1D.CurrentAxes = h_axis1D(2);
end
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            h_stairs(index_axis,2) = stairs(t(index_axis,1:index_stop(index_axis)),V(index_axis,1:index_stop(index_axis)),'Color',color_line(index_axis,:),'LineWidth',size_traj);
            h_start(index_axis,2) = scatter(0,State_start(index_axis,2),size_point,color_startpoint,'Marker','o','MarkerFaceColor',color_startpoint);
            for index_waypoint = 1:1:num_waypoints
                if (index_waypoint == num_waypoints)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                    if (~isinf(V_max(index_axis,index_waypoint)) && ~isinf(V_max(index_axis,index_waypoint-1)))
                        line([T_old,T_old],[V_max(index_axis,index_waypoint),V_max(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit);
                    else
                        line([T_old,T_old],[min(V_max(index_axis,index_waypoint),V_max(index_axis,index_waypoint-1)),axislimits_1D(2,4)],'Color',color_limit,'LineWidth',size_limit);
                    end
                    if (~isinf(V_min(index_axis,index_waypoint)) && ~isinf(V_min(index_axis,index_waypoint-1)))
                        line([T_old,T_old],[V_min(index_axis,index_waypoint),V_min(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit); 
                    else
                        line([T_old,T_old],[max(V_min(index_axis,index_waypoint),V_min(index_axis,index_waypoint-1)),axislimits_1D(2,3)],'Color',color_limit,'LineWidth',size_limit);
                    end
                end
                h_limits_1(index_axis,1,index_waypoint) = line([T_old,T],[V_max(index_axis,index_waypoint),V_max(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                h_limits_2(index_axis,1,index_waypoint) = line([T_old,T],[V_min(index_axis,index_waypoint),V_min(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                patch([T_old,T_old,T,T],[V_max(index_axis,index_waypoint),max(axislimits_1D(2,4),V_max(index_axis,index_waypoint)),max(axislimits_1D(2,4),V_max(index_axis,index_waypoint)),V_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old,T_old,T,T],[V_min(index_axis,index_waypoint),min(axislimits_1D(2,3),V_min(index_axis,index_waypoint)),min(axislimits_1D(2,3),V_min(index_axis,index_waypoint)),V_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                line([T_old T],[0 0],'Color','black','LineStyle',':','Linewidth',size_segment);
                h_waypoints_1(index_axis,2,index_waypoint) = scatter(0,Waypoints(index_axis,2,index_waypoint)+Waypoints(index_axis,4,index_waypoint),size_point,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                h_waypoints_2(index_axis,2,index_waypoint) = scatter(T,Waypoint_evolved(index_axis,2),size_point,color,'Marker','o','MarkerFaceColor',color);
            end
            for index_times = 2:1:num_times(index_axis)
                t_marker = J_setp_struct(index_axis).time(index_times) - ts_rollout;
                line([t_marker, t_marker],[axislimits_1D(2,3), axislimits_1D(2,4)],'Color','black','LineStyle',':','Linewidth',size_segment);
                if (b_plot_markers(2) == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),t_marker);
                    h_marker(index_axis,2,index_times) = scatter(t_marker,V_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
        end
    end
if (~exist('h_axis1D','var') || size(h_axis1D,2) < 3 || ~isvalid(h_axis1D(3)))
    h_axis1D(3) = subplot(4,1,3);
else
    h_figure1D.CurrentAxes = h_axis1D(3);
end
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            h_stairs(index_axis,3) = stairs(t(index_axis,1:index_stop(index_axis)),A(index_axis,1:index_stop(index_axis)),'Color',color_line(index_axis,:),'LineWidth',size_traj);
            h_start(index_axis,3) = scatter(0,State_start(index_axis,3),size_point,color_startpoint,'Marker','o','MarkerFaceColor',color_startpoint);
            for index_waypoint = 1:1:num_waypoints
                if (index_waypoint == num_waypoints)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                    if (~isinf(A_max(index_axis,index_waypoint)) && ~isinf(A_max(index_axis,index_waypoint-1)))
                        line([T_old T_old],[A_max(index_axis,index_waypoint),A_max(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit);
                    else
                        line([T_old T_old],[min(A_max(index_axis,index_waypoint),A_max(index_axis,index_waypoint-1)),axislimits_1D(3,4)],'Color',color_limit,'LineWidth',size_limit);
                    end
                    if (~isinf(A_min(index_axis,index_waypoint)) && ~isinf(A_min(index_axis,index_waypoint-1)))
                        line([T_old T_old],[A_min(index_axis,index_waypoint),A_min(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit); 
                    else
                        line([T_old T_old],[max(A_min(index_axis,index_waypoint),A_min(index_axis,index_waypoint-1)),axislimits_1D(3,3)],'Color',color_limit,'LineWidth',size_limit);
                    end
                end
                h_limits_1(index_axis,2,index_waypoint) = line([T_old T],[A_max(index_axis,index_waypoint),A_max(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                h_limits_2(index_axis,2,index_waypoint) = line([T_old T],[A_min(index_axis,index_waypoint),A_min(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                patch([T_old, T_old, T, T],[A_max(index_axis,index_waypoint),max(axislimits_1D(3,4),A_max(index_axis,index_waypoint)),max(axislimits_1D(3,4),A_max(index_axis,index_waypoint)),A_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old, T_old, T, T],[A_min(index_axis,index_waypoint),min(axislimits_1D(3,3),A_min(index_axis,index_waypoint)),min(axislimits_1D(3,3),A_min(index_axis,index_waypoint)),A_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                line([T_old T],[0 0],'Color','black','LineStyle',':','Linewidth',size_segment);
                h_waypoints_1(index_axis,3,index_waypoint) = scatter(0,Waypoints(index_axis,3,index_waypoint),size_point,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                h_waypoints_2(index_axis,3,index_waypoint) = scatter(T,Waypoint_evolved(index_axis,3),size_point,color,'Marker','o','MarkerFaceColor',color);
            end
            for index_times = 2:1:num_times(index_axis)
                t_marker = J_setp_struct(index_axis).time(index_times) - ts_rollout;
                line([t_marker, t_marker],[axislimits_1D(3,3), axislimits_1D(3,4)],'Color','black','LineStyle',':','Linewidth',size_segment);
                if (b_plot_markers(3) == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),t_marker);
                    h_marker(index_axis,3,index_times) = scatter(t_marker,A_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
        end
    end
if (~exist('h_axis1D','var') || size(h_axis1D,2) < 4 || ~isvalid(h_axis1D(4)))
    h_axis1D(4) = subplot(4,1,4);
else
    h_figure1D.CurrentAxes = h_axis1D(4);
end
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            h_stairs(index_axis,4) = stairs(t(index_axis,1:index_stop(index_axis)-1),J(index_axis,1:index_stop(index_axis)-1),'Color',color_line(index_axis,:),'LineWidth',size_traj);
            for index_waypoint = 1:1:num_waypoints
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                   if (~isinf(J_max(index_axis,index_waypoint)) && ~isinf(J_max(index_axis,index_waypoint-1)))
                        line([T_old T_old],[J_max(index_axis,index_waypoint),J_max(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit);
                    else
                        line([T_old T_old],[min(J_max(index_axis,index_waypoint),J_max(index_axis,index_waypoint-1)),axislimits_1D(4,4)],'Color',color_limit,'LineWidth',size_limit);
                    end
                    if (~isinf(J_min(index_axis,index_waypoint)) && ~isinf(J_min(index_axis,index_waypoint-1)))
                        line([T_old T_old],[J_min(index_axis,index_waypoint),J_min(index_axis,index_waypoint-1)],'Color',color_limit,'LineWidth',size_limit); 
                    else
                        line([T_old T_old],[max(J_min(index_axis,index_waypoint),J_min(index_axis,index_waypoint-1)),axislimits_1D(4,3)],'Color',color_limit,'LineWidth',size_limit);
                    end
                end
                h_limits_1(index_axis,3,index_waypoint) = line([T_old T],[J_max(index_axis,index_waypoint),J_max(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                h_limits_2(index_axis,3,index_waypoint) = line([T_old T],[J_min(index_axis,index_waypoint),J_min(index_axis,index_waypoint)],'Color',color_limit,'LineWidth',size_limit);
                patch([T_old, T_old, T, T],[J_max(index_axis,index_waypoint),max(axislimits_1D(4,4),J_max(index_axis,index_waypoint)),max(axislimits_1D(4,4),J_max(index_axis,index_waypoint)),J_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old, T_old, T, T],[J_min(index_axis,index_waypoint),min(axislimits_1D(4,3),J_min(index_axis,index_waypoint)),min(axislimits_1D(4,3),J_min(index_axis,index_waypoint)),J_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                line([T_old T],[0 0],'Color','black','LineStyle',':','Linewidth',size_segment);
            end
            for index_times = 2:1:num_times(index_axis)
                t_marker = J_setp_struct(index_axis).time(index_times) - ts_rollout;
                line([t_marker, t_marker],[axislimits_1D(4,3), axislimits_1D(4,4)],'Color','black','LineStyle',':','Linewidth',size_segment);
                if (b_plot_markers(4) == true)
                    [P_switch, V_switch, A_switch, J_switch] = evaluate_to_time(State_start(:,1),State_start(:,2),State_start(:,3),J_setp_struct(index_axis),t_marker);
                    h_marker(index_axis,4,index_times) = scatter(t_marker,J_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
        end
    end

linkaxes(h_axis1D,'x');

ylabel(h_axis1D(1),'Position (m)');
ylabel(h_axis1D(2),'Velocity (m/s)');
ylabel(h_axis1D(3),'Acceleration (m/s^2)');
ylabel(h_axis1D(4),'Jerk (m/s^3)');
xlabel(h_axis1D(4),'Time (s)');

if (nnz(b_plot_axis) > 1)
    legend(h_stairs(b_plot_axis,1),axisnames(b_plot_axis),'AutoUpdate','off','Location','Southeast');
end

for index_subplot = 1:4
    handle = h_axis1D(index_subplot);
    
    handle.YTick = axislimits_1D(index_subplot,3):(axislimits_1D(index_subplot,4)-axislimits_1D(index_subplot,3))/4:axislimits_1D(index_subplot,4);
    xtickformat(handle,tickformat{1});
    ytickformat(handle,tickformat{2});
    box(handle,'on');
    
    if (index_subplot ~= 4)
        handle.XTickLabel = [];
    end
    handle.FontSize = size_font;
    
    axis(handle,axislimits_1D(index_subplot,:));

    for index_axis = num_axes:-1:1
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            uistack(h_stairs(index_axis,index_subplot),'top');
            if (b_plot_markers(index_subplot) == true)
                for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                    uistack(h_marker(index_axis,index_subplot,index_times),'top');
                end
            end
            if (index_subplot ~= 4)  
                uistack(h_start(index_axis,index_subplot),'top');
                for index_waypoint = 1:num_waypoints
                    uistack(h_waypoints_1(index_axis,index_subplot,index_waypoint),'top');
                    uistack(h_waypoints_2(index_axis,index_subplot,index_waypoint),'top');
                    
                    h_waypoints_1(index_axis,index_subplot,index_waypoint).MarkerEdgeColor = 'none';
                end
            end
        end
    end
end

inset = [0,0,0.01,0]; %leave a little margin for points
for index_subplot = 1:4
    test   = h_axis1D(index_subplot).TightInset;
    inset  = [max(test(1),inset(1)),max(test(2),inset(2)),max(test(3),inset(3)),min(test(4),inset(4))];
end
for index_subplot = 1:4
    pos    = h_axis1D(index_subplot).Position;
    h_axis1D(index_subplot).Position = [inset(1),pos(2), 1-inset(1)-inset(3), pos(4)];
end


if (b_record_video == true)
    if (index_iteration == 1)
        M = getframe(h_figure1D);
    else
        M(size(M,2)+1) = getframe(h_figure1D);
    end
end
