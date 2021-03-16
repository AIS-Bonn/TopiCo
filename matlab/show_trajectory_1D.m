%------------------------------------------------------------------------
% File:       show_trajectory_1D.m
% Version:    2018-08-29 10:37:04
% Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
% Package:    opt_control (https://github.com/AIS-Bonn/opt_control)
% License:    BSD
%------------------------------------------------------------------------

% Software License Agreement (BSD License)
% Copyright (c) 2018, Computer Science Institute VI, University of Bonn
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
%------------------------------------------------------------------------

if (~exist('resolution','var'))
    resolution = [-2880 1620 2880 1620];
end
if (~exist('size_font','var'))
    size_font = 16;
end
if (~exist('b_plot_marker','var'))
    b_plot_marker = true;
end
if (~exist('b_record_video','var'))
    b_record_video = false;
end
if (~exist('T_viapoints','var'))
    T_viapoints = [];
end
if (~exist('Obstacles','var'))
    Obstacles = [];
end
if (~exist('b_plot_axis','var'))
    b_plot_axis = ones(num_axes,1);
end

color_start    = [0.0 0.7 0.0];
color_waypoint = [0.0 0.0 0.7];
color_endpoint = [0.7 0.0 0.0];
color_viapoint = [0.5 0.5 0.5];
color_obstacle = [0.5 0.5 0.5];
color_limit    = [1.0 0.0 0.0];
color_marker   = [0.0 0.0 0.0];
alpha_limit    = 0.05;
size_marker    = 30;
size_waypoints = 150;



%figure;
%clf;
set(gcf,'units','pixels','Resize','off','Position',resolution);
%set(gcf,'MenuBar','none');

num_axes = size(Waypoints,1);
num_trajectories = size(Waypoints,3);

axislimits = zeros(4,4);
axislimits(:,1) = 0;
axislimits(:,2) = T_rollout;
for index_axis = 1:num_axes
    if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
        axislimits(1,3) = min(axislimits(1,3),min(P(index_axis,:)));
        axislimits(1,4) = max(axislimits(1,4),max(P(index_axis,:)));
        axislimits(2,3) = min(axislimits(2,3),min(V(index_axis,:)));
        axislimits(2,4) = max(axislimits(2,4),max(V(index_axis,:)));
        axislimits(3,3) = min(axislimits(3,3),min(A(index_axis,:)));
        axislimits(3,4) = max(axislimits(3,4),max(A(index_axis,:)));
        axislimits(4,3) = min(axislimits(4,3),min(J_setp_struct(index_axis).signals.values));
        axislimits(4,4) = max(axislimits(4,4),max(J_setp_struct(index_axis).signals.values));
    end
end
axislimits(:,3) = axislimits(:,3)-(axislimits(:,4)-axislimits(:,3))/10;
axislimits(:,4) = axislimits(:,4)+(axislimits(:,4)-axislimits(:,3))/10;

h1 = subplot(4,1,1);
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            stairs(t(index_axis,:),P(index_axis,:),'Color','black');
            scatter(0,State_start(index_axis,1),size_waypoints,color_start,'Marker','o','MarkerFaceColor',color_start);
            for index_waypoint = 1:1:num_trajectories
                if (index_waypoint == num_trajectories)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                scatter(0,Waypoints(index_axis,1,index_waypoint),size_waypoints,color,'Marker','o','MarkerFaceColor',color);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                scatter(T,Waypoint_evolved(index_axis,1),size_waypoints,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                xt = @(t) Waypoints(index_axis,1,index_waypoint)+Waypoints(index_axis,4,index_waypoint).*t+1/2.*Waypoints(index_axis,5,index_waypoint).*t.^2;
                fplot(xt,[0 T],'Color',color,'LineStyle',':');
            end
            for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                line([J_setp_struct(index_axis).time(index_times,1) J_setp_struct(index_axis).time(index_times,1)],[axislimits(1,3) axislimits(1,4)],'Color','black','LineStyle',':');
                %text(J_setp_struct(index_axis).time(index_times-1) + (J_setp_struct(index_axis).time(index_times) - J_setp_struct(index_axis).time(index_times-1)) / 2,axislimits(1,4),arabic_to_roman(index_times-1),'FontSize',size_font,'FontName','Times','HorizontalAlignment','center','VerticalAlignment','bottom');
                text(J_setp_struct(index_axis).time(index_times-1) + (J_setp_struct(index_axis).time(index_times) - J_setp_struct(index_axis).time(index_times-1)) / 2,axislimits(1,4),num2str(index_times-1),'FontSize',size_font,'FontName','Times','HorizontalAlignment','center','VerticalAlignment','bottom');
                if (b_plot_marker == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),J_setp_struct(index_axis).time(index_times));
                    scatter(J_setp_struct(index_axis).time(index_times),P_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
            for index_viapoint = 1:size(T_viapoints,2)
                [P_viapoint,V_viapoint,A_viapoint,J_viapoint] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct,sum(T_viapoints(index_viapoint)));
                scatter(T_viapoints(index_viapoint),P_viapoint(index_axis),size_waypoints,color_viapoint,'Marker','o','MarkerFaceColor',color_viapoint);
            end    
%             if ~(isempty(Obstacles))
%                 t = linspace(axislimits(1,1),axislimits(1,2),100);
%                 for index_Obstacle = 1:size(Obstacles,3)
%                     Obstacle = Obstacles(:,:,index_Obstacle);
%                     for index_t = 1:size(t,2)
%                         Obstacle_evolved = evolve_obstacles(Obstacle,t(index_t));
%                         y_1(index_t) = Obstacle_evolved(index_axis,1,1);
%                         y_2(index_t) = Obstacle_evolved(index_axis,5,1);
%                     end
%                     alpha_patch = 0.2;
%                     xt = @(t) Obstacle(index_axis,1)-MAV_margin(index_axis)-MAV_d(index_axis)+Obstacle(index_axis,2).*t+1/2.*Obstacle(index_axis,3).*t.^2+1/6.*Obstacle(index_axis,4).*t.^3;
%                     fplot(xt,[axislimits(1,1) axislimits(1,2)],'Color',[0.5,0.5,0.5],'LineStyle',':','LineWidth',2);
%                     xt = @(t) Obstacle(index_axis,5)+MAV_margin(index_axis)+MAV_d(index_axis)+Obstacle(index_axis,6).*t+1/2.*Obstacle(index_axis,7).*t.^2+1/6.*Obstacle(index_axis,8).*t.^3;
%                     fplot(xt,[axislimits(1,1) axislimits(1,2)],'Color',[0.5,0.5,0.5],'LineStyle',':','LineWidth',2);
%                 end
%                 fill([t t(end:-1:1)],[y_1 y_2(end:-1:1)],color_obstacle,'FaceAlpha',alpha_patch);
%             end
        end
    end
h2 = subplot(4,1,2);
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true) 
            stairs(t(index_axis,:),V(index_axis,:),'Color','black');
            scatter(0,State_start(index_axis,2),size_waypoints,color_start,'Marker','o','MarkerFaceColor',color_start);
            for index_waypoint = 1:1:num_trajectories
                if (index_waypoint == num_trajectories)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                    line([T_old T_old],[V_max(index_axis,index_waypoint),V_max(index_axis,index_waypoint-1)],'Color',color_limit); 
                end
                line([T_old T],[V_max(index_axis,index_waypoint),V_max(index_axis,index_waypoint)],'Color',color_limit);
                line([T_old T],[V_min(index_axis,index_waypoint),V_min(index_axis,index_waypoint)],'Color',color_limit);
                patch([T_old, T_old, T, T],[V_max(index_axis,index_waypoint),max(axislimits(2,4),V_max(index_axis,index_waypoint)),max(axislimits(2,4),V_max(index_axis,index_waypoint)),V_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old, T_old, T, T],[V_min(index_axis,index_waypoint),min(axislimits(2,3),V_min(index_axis,index_waypoint)),min(axislimits(2,3),V_min(index_axis,index_waypoint)),V_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                scatter(0,Waypoints(index_axis,2,index_waypoint)+Waypoints(index_axis,4,index_waypoint),size_waypoints,color,'Marker','o','MarkerFaceColor',color);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                scatter(T,Waypoint_evolved(index_axis,2),size_waypoints,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                xt = @(t) Waypoints(index_axis,2,index_waypoint)+Waypoints(index_axis,4,index_waypoint)+Waypoints(index_axis,5,index_waypoint).*t;
                fplot(xt,[0 T],'Color',color,'LineStyle',':');
            end
            for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                line([J_setp_struct(index_axis).time(index_times,1) J_setp_struct(index_axis).time(index_times,1)],[axislimits(2,3) axislimits(2,4)],'Color','black','LineStyle',':');
                if (b_plot_marker == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),J_setp_struct(index_axis).time(index_times));
                    scatter(J_setp_struct(index_axis).time(index_times),V_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
            for index_viapoint = 1:size(T_viapoints,2)
                [P_viapoint,V_viapoint,A_viapoint,J_viapoint] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct,sum(T_viapoints(index_viapoint)));
                scatter(T_viapoints(index_viapoint),V_viapoint(index_axis),size_waypoints,color_viapoint,'Marker','o','MarkerFaceColor',color_viapoint);
            end
        end
    end
h3 = subplot(4,1,3);
   hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            stairs(t(index_axis,:),A(index_axis,:),'Color','black');
            scatter(0,State_start(index_axis,3),size_waypoints,color_start,'Marker','o','MarkerFaceColor',color_start);
            for index_waypoint = 1:1:num_trajectories
                if (index_waypoint == num_trajectories)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                    line([T_old T_old],[A_max(index_axis,index_waypoint),A_max(index_axis,index_waypoint-1)],'Color',color_limit); 
                end
                line([T_old T],[A_max(index_axis,index_waypoint),A_max(index_axis,index_waypoint)],'Color',color_limit);
                line([T_old T],[A_min(index_axis,index_waypoint),A_min(index_axis,index_waypoint)],'Color',color_limit);
                patch([T_old, T_old, T, T],[A_max(index_axis,index_waypoint),max(axislimits(3,4),A_max(index_axis,index_waypoint)),max(axislimits(3,4),A_max(index_axis,index_waypoint)),A_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old, T_old, T, T],[A_min(index_axis,index_waypoint),min(axislimits(3,3),A_min(index_axis,index_waypoint)),min(axislimits(3,3),A_min(index_axis,index_waypoint)),A_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                line([T_old T],[0 0],'Color','black','LineStyle',':');
                scatter(0,Waypoints(index_axis,3,index_waypoint),size_waypoints,color,'Marker','o','MarkerFaceColor',color);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                scatter(T,Waypoint_evolved(index_axis,3),size_waypoints,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                xt = @(t) Waypoints(index_axis,3,index_waypoint)+Waypoints(index_axis,5,index_waypoint)+0.*t;
                fplot(xt,[0 T],'Color',color,'LineStyle',':');
            end
            for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                line([J_setp_struct(index_axis).time(index_times,1) J_setp_struct(index_axis).time(index_times,1)],[axislimits(3,3) axislimits(3,4)],'Color','black','LineStyle',':');
                if (b_plot_marker == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),J_setp_struct(index_axis).time(index_times));
                    scatter(J_setp_struct(index_axis).time(index_times),A_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
            for index_viapoint = 1:size(T_viapoints,2)
                [P_viapoint,V_viapoint,A_viapoint,J_viapoint] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct,sum(T_viapoints(index_viapoint)));
                scatter(T_viapoints(index_viapoint),A_viapoint(index_axis),size_waypoints,color_viapoint,'Marker','o','MarkerFaceColor',color_viapoint);
            end
        end
    end
h4 = subplot(4,1,4);
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            stairs(t(index_axis,:),J(index_axis,:),'Color','black');
            for index_waypoint = 1:1:num_trajectories
                T_old = sum(T_waypoints(index_axis,1:index_waypoint-1));
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                if (index_waypoint > 1)
                    line([T_old T_old],[J_max(index_axis,index_waypoint),J_max(index_axis,index_waypoint-1)],'Color',color_limit); 
                end
                line([T_old T],[J_max(index_axis,index_waypoint),J_max(index_axis,index_waypoint)],'Color',color_limit);
                line([T_old T],[J_min(index_axis,index_waypoint),J_min(index_axis,index_waypoint)],'Color',color_limit);
                patch([T_old, T_old, T, T],[J_max(index_axis,index_waypoint),max(axislimits(4,4),J_max(index_axis,index_waypoint)),max(axislimits(4,4),J_max(index_axis,index_waypoint)),J_max(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
                patch([T_old, T_old, T, T],[J_min(index_axis,index_waypoint),min(axislimits(4,3),J_min(index_axis,index_waypoint)),min(axislimits(4,3),J_min(index_axis,index_waypoint)),J_min(index_axis,index_waypoint)],color_limit,'EdgeAlpha',0,'FaceAlpha',alpha_limit);
            end
            for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                line([J_setp_struct(index_axis).time(index_times,1) J_setp_struct(index_axis).time(index_times,1)],[axislimits(4,3) axislimits(4,4)],'Color','black','LineStyle',':');
                if (b_plot_marker == true)
                    [P_switch V_switch A_switch J_switch] = evaluate_to_time(State_start(:,1),State_start(:,2),State_start(:,3),J_setp_struct(index_axis),J_setp_struct(index_axis).time(index_times));
                    scatter(J_setp_struct(index_axis).time(index_times),J_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
        end
    end

    
linkaxes([h1 h2 h3 h4],'x');

ylabel(h1,'Position (m)');
ylabel(h2,'Velocity (m/s)');
ylabel(h3,'Acceleration (m/s^2)');
ylabel(h4,'Jerk (m/s^3)');
xlabel(h4,'Time (s)');

for index_subplot = 1:4
    switch index_subplot
        case 1
            handle = h1;
        case 2
            handle = h2;
        case 3
            handle = h3;
        case 4
            handle = h4;
    end
    
    set(handle,'YTick',axislimits(index_subplot,3):(axislimits(index_subplot,4)-axislimits(index_subplot,3))/4:axislimits(index_subplot,4));
    set(handle,'YTicklabel', num2str(get(handle, 'YTick')', '%.1f'));
    if (index_subplot ~= 4)
        set(handle,'XTickLabel',[]);
    end
    set(handle,'FontSize',size_font);
    
    axis(handle,axislimits(index_subplot,:));
    
    margin = 0.005;
    inset = get(handle,'TightInset');
    pos   = get(handle,'Position');
    set(handle,'Position',[inset(1),pos(2), 1-inset(1)-inset(3)-margin, pos(4)],'FontSize',size_font);

end

if (b_record_video == true)
    if (index_iteration == 1)
        M = getframe(gcf);
    else
        M(size(M,2)+1) = getframe(gcf);
    end
end
