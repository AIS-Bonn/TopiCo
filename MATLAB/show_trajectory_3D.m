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

assert(num_axes >= 3,'Error: For show_trajectory_3D, trajectory has to have at least 3 axes!')

show_trajectory_3D_defaults;
if (~exist('h_figure3D','var') || ~isvalid(h_figure3D))
    h_figure3D = figure('Color','w','Units','pixels','Resize','off');
    for i = 1:1:10 % MATLAB Bug
        h_figure3D.Position = resolution;
        drawnow;
    end
else
    figure(h_figure3D);
end
if (b_clf == true)
    clf;
end
if (~exist('axislimits_3D','var'))
    minsize = [0.0;  0.0;  0.2];
    axislimits_3D = [min(P(1,:))-minsize(1), max(P(1,:))+minsize(1); min(P(2,:))-minsize(2), max(P(2,:))+minsize(2); min(P(3,:))-minsize(3), max(P(3,:))+minsize(3)];
    axislimits_3D(1,1) = min(axislimits_3D(1,1),min(Waypoints(1,1,:)));
    axislimits_3D(1,2) = max(axislimits_3D(1,2),max(Waypoints(1,1,:)));
    axislimits_3D(2,1) = min(axislimits_3D(2,1),min(Waypoints(2,1,:)));
    axislimits_3D(2,2) = max(axislimits_3D(2,2),max(Waypoints(2,1,:)));
    axislimits_3D(3,1) = min(axislimits_3D(3,1),min(Waypoints(3,1,:)));
    axislimits_3D(3,2) = max(axislimits_3D(3,2),max(Waypoints(3,1,:)));
end
if (index_infl_axes ~= 0)
    if (index_infl_axes == 1) %MAV at start in all rotations visible
        MAV_max = 0.001 * ones(1,size(P,2));
        MAV_max(1) = max(vecnorm(MAV_d(1:3,:),2,1));
    elseif (index_infl_axes == 2) %MAV everywhere in all rotations visible
        MAV_max = max(vecnorm(MAV_d(1:3,:),2,1));
    elseif (index_infl_axes == 3) %Just the path
        MAV_max = 0;    
    end
    axislimits_3D = [min(min(P(1,:)-MAV_max),axislimits_3D(1)), max(max(P(1,:)+MAV_max),axislimits_3D(2)); min(min(P(2,:)-MAV_max),axislimits_3D(3)), max(max(P(2,:)+MAV_max),axislimits_3D(4)); min(min(P(3,:)-MAV_max),axislimits_3D(5)), max(max(P(3,:)+MAV_max),axislimits_3D(6))];
end

if (~exist('h_axis3D','var') || ~isvalid(h_axis3D))
    h_axis3D = axes(h_figure3D,'Box','on','XGrid','on','YGrid','on','ZGrid','on','FontSize',size_font,'XLim',axislimits_3D(1,:),'YLim',axislimits_3D(2,:),'ZLim',axislimits_3D(3,:),'DataAspectRatio',[1,1,1]);
    xlabel(h_axis3D,'X-Position (m)','FontSize',size_font);
    ylabel(h_axis3D,'Y-Position (m)','FontSize',size_font);
    zlabel(h_axis3D,'Z-Position (m)','FontSize',size_font);
    xtickformat(h_axis3D,tickformat{1});
    ytickformat(h_axis3D,tickformat{2});
    ztickformat(h_axis3D,tickformat{3});
    xticks(h_axis3D,axislimits_3D(1,1):ticks(1):axislimits_3D(1,2));
    yticks(h_axis3D,axislimits_3D(2,1):ticks(2):axislimits_3D(2,2));
    zticks(h_axis3D,axislimits_3D(3,1):ticks(3):axislimits_3D(3,2));
    hold all;
end

[x,y,z] = sphere;
x_sphere = x*size_sphere;
y_sphere = y*size_sphere;
z_sphere = z*size_sphere;


%Trajectory Rollout
switch index_deriv
    case 1
        deriv_magnitude = vecnorm(V(1:3,:));
    case 2 
        deriv_magnitude = vecnorm(A(1:3,:));
    case 3
        deriv_magnitude = vecnorm(J(1:3,:));
end
if (index_deriv ~= 0)
     scatter3(P(1,:),P(2,:),P(3,:),size_deriv,deriv_magnitude,'filled','MarkerFaceAlpha',alpha_deriv);
else
    plot3(P(1,:),P(2,:),P(3,:),'Color','k','LineStyle',':','LineWidth',size_traj);
end

%Startpoint
surf(x_sphere+State_start(1,1),y_sphere+State_start(2,1),z_sphere+State_start(3,1),'FaceColor',color_startpoint,'LineStyle','none');


%Waypoints Start
for index_waypoint = 1:1:num_waypoints-1
    if (nnz(Waypoints(:,4:5,index_waypoint)) > 0)
        scatter3(Waypoints(1,1,index_waypoint),Waypoints(2,1,index_waypoint),Waypoints(3,1,index_waypoint),pi/4*150,color_startpoint,'Marker','s','MarkerFaceColor',color_waypoint,'MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
    end
end

%Endpoint Start
scatter3(Waypoints(1,1,end),Waypoints(2,1,end),Waypoints(3,1,end),pi/4*150,color_endpoint,'Marker','s','MarkerFaceColor',color_endpoint,'MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);

%Predicted Waypoint Interception Positions
for index_waypoint = 1:1:num_waypoints-1
    if (nnz(Waypoints(:,4:5,index_waypoint)) > 0)
        T = max(sum(T_waypoints(:,1:index_waypoint),2));
        Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
        scatter3(Waypoint_evolved(1,1,1),Waypoint_evolved(2,1,1),Waypoint_evolved(3,1,1),size_point,color_waypoint,'Marker','o','MarkerFaceColor',color_waypoint,'MarkerFaceAlpha',0.0,'LineWidth',size_ring);
        if (b_draw_text(1) == true)
            text(Waypoint_evolved(1,1,1),Waypoint_evolved(2,1,1),Waypoint_evolved(3,1,1),['T = ',num2str(T,'%.3f'),'s'],'FontSize',size_font,'VerticalAlignment','bottom','HorizontalAlignment','center');
        end
    end
end

%Predicted Endpoint Interception Position
if (nnz(Waypoints(:,4:5,end)) > 0)
    T = max(sum(T_waypoints,2));
    Waypoint_evolved = evolve_waypoints(Waypoints(:,:,end),T);
    scatter3(Waypoint_evolved(1,1,1),Waypoint_evolved(2,1,1),Waypoint_evolved(3,1,1),size_point,color_endpoint,'Marker','o','MarkerFaceColor',color_endpoint,'MarkerFaceAlpha',0.0,'LineWidth',size_ring);
end
    
%Total trajectory time
if (b_draw_text(2) == true)
    offset = zeros(3,1);
    text(Waypoint_evolved(1,1,1)+offset(1),Waypoint_evolved(2,1,1)+offset(2),Waypoint_evolved(3,1,1)+offset(3),['T = ',num2str(T,'%.3f'),'s'],'FontSize',size_font,'VerticalAlignment','bottom','HorizontalAlignment','center');
end

%Waypoint Path
for index_waypoint = 1:1:num_waypoints-1
    xt = @(t) Waypoints(1,1,index_waypoint)+Waypoints(1,4,index_waypoint).*t+1/2.*Waypoints(1,5,index_waypoint).*t.^2;
    yt = @(t) Waypoints(2,1,index_waypoint)+Waypoints(2,4,index_waypoint).*t+1/2.*Waypoints(2,5,index_waypoint).*t.^2;
    zt = @(t) Waypoints(3,1,index_waypoint)+Waypoints(3,4,index_waypoint).*t+1/2.*Waypoints(3,5,index_waypoint).*t.^2;
    fplot3(xt,yt,zt,[0 sum(max(T_waypoints))],'Color',color_waypoint,'LineStyle',':','LineWidth',size_path);
end

%Endpoint Path
xt = @(t) Waypoints(1,1,end)+Waypoints(1,4,end).*t+1/2.*Waypoints(1,5,end).*t.^2;
yt = @(t) Waypoints(2,1,end)+Waypoints(2,4,end).*t+1/2.*Waypoints(2,5,end).*t.^2;
zt = @(t) Waypoints(3,1,end)+Waypoints(3,4,end).*t+1/2.*Waypoints(3,5,end).*t.^2;
fplot3(xt,yt,zt,[0 sum(max(T_waypoints))],'Color',color_endpoint,'LineStyle',':','LineWidth',size_path);


%Trajectory Rollout in buffer
for index_buff = 2:size_buff
    switch index_deriv
        case 1
            scatter3(Buff(index_buff).P(1,:),Buff(index_buff).P(2,:),Buff(index_buff).P(3,:),size_deriv,vecnorm(Buff(index_buff).V(1:3,:)),'filled','MarkerFaceAlpha',alpha_deriv_buff);
        case 2 
            scatter3(Buff(index_buff).P(1,:),Buff(index_buff).P(2,:),Buff(index_buff).P(3,:),size_deriv,vecnorm(Buff(index_buff).A(1:3,:)),'filled','MarkerFaceAlpha',alpha_deriv_buff);
        case 3
            scatter3(Buff(index_buff).P(1,:),Buff(index_buff).P(2,:),Buff(index_buff).P(3,:),size_deriv,vecnorm(Buff(index_buff).J(1:3,:)),'filled','MarkerFaceAlpha',alpha_deriv_buff);
        otherwise
            plot3(Buff(index_buff).P(1,:),Buff(index_buff).P(2,:),Buff(index_buff).P(3,:),'Color','k','LineStyle',':','LineWidth',size_traj);
    end
end

if (index_deriv ~= 0)
    if (exist('h_colorbar','var'))
        delete(h_colorbar);
    else
        limits_cb_old = [min(deriv_magnitude),max(deriv_magnitude)];
    end
    colormap_temp = jet(256);
    colormap(colormap_temp(1:200,:));
    
    colorbar_limits = [min([deriv_magnitude,limits_cb_old(1)]),max([deriv_magnitude,limits_cb_old(2)])];
    h_colorbar = colorbar(h_axis3D,'FontSize',size_font,'Limits',colorbar_limits,'LimitsMode','manual');
    caxis(h_colorbar.Limits);
    
    labelcell = {'Velocity (m/s)','Acceleration (m/s^2)','Jerk (m/s^3)'};
    ylabel(h_colorbar,labelcell{index_deriv});
    %h_colorbar.Limits = [0.0 h_colorbar.Limits(2)];
    
    switch tickformat{4}(2:3)
        case '.0'
            diff = 1.0;
        case '.1'
            diff = 0.1;
        otherwise
            disp('Warning. Check tickformat(4)!');
    end
    h_colorbar.Ticks = 0:diff:h_colorbar.Limits(2);
    h_colorbar.TickLabels = num2str(h_colorbar.Ticks',tickformat{4});
    drawnow;
end

h_axis3D.View = viewangle;

if (exist('p_outer_override','var'))
    h_axis3D.OuterPosition = p_outer_override;
else
    for i = 1:1:10 %MATLAB Bug
        InSet = h_axis3D.TightInset;
        h_axis3D.Position = [InSet(1:2), 1-InSet(1)-InSet(3)-colorbar_margin, 1-InSet(2)-InSet(4)];
        drawnow;
    end
    h_colorbar.Position(3) = colorbar_width;
end
if (h_axis3D.Position(2) ~= 0 || h_axis3D.Position(4) ~= 1)
    disp('Warning: Check Aspect Ratio!');
end

video_min = t_video_start * fps;
if (b_video_rollout == true)
    video_max = floor(sum(max(T_waypoints)) * fps);
else
    video_max = video_min;
end


angle = rad2deg(atan2(V(2,1 + floor(t_video_start / ts_rollout)),V(1,1 + floor(t_video_start / ts_rollout))))-90;


for iteration_video = video_min:video_max

    HandleArray = cell(0,0);

    T_animation = iteration_video / fps;
    index = 1 + floor(iteration_video / ts_rollout / fps);

    if (b_rotate_velocity == true)
        angle_curr = rad2deg(atan2(V(2,index),V(1,index)))-90;
        angle = angle + max(min((angle_curr - angle) * angle_factor,angle_max_vel),angle_min_vel) / fps;
        h_axis3D.View = [angle,viewangle(2)];
    else
        h_axis3D.View = viewangle+viewspeed*T_animation;
    end

    %MAVs on Trajectory
    if (mod(T_animation,t_draw_MAV) < 1/fps*0.99)
        draw_MAV(P(:,index),A(:,index)-A_global,select_MAV(MAV_d,T_waypoints,T_animation),g,alpha_MAV_traj,MAV_zoom,index_MAV_bb,size_MAV_sphere,size_MAV_line,size_MAV_line_bb);
    end

    %MAV for current point
    HandleArray{size(HandleArray,2)+1} = draw_MAV(P(:,index),A(:,index)-A_global,select_MAV(MAV_d,T_waypoints,T_animation),g,alpha_MAV,MAV_zoom,index_MAV_bb,size_MAV_sphere,size_MAV_line,size_MAV_line_bb);

    %MAV in buffer
    for index_buff = 2:size_buff
        if (~isempty(Buff(index_buff).P))
             HandleArray{size(HandleArray,2)+1} = draw_MAV(Buff(index_buff).P,Buff(index_buff).A-A_global,select_MAV(MAV_d,Buff(index_buff).T_waypoints,T_animation),g,alpha_MAV_buff,MAV_zoom,index_MAV_bb,size_MAV_sphere,size_MAV_line,size_MAV_line_bb);
        end
    end

    %Blue Waypoints
    for index_waypoint = 1:1:num_waypoints-1
        Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T_animation);
        HandleArray{size(HandleArray,2)+1} = surf(x_sphere+Waypoint_evolved(1,1,1),y_sphere+Waypoint_evolved(2,1,1),z_sphere+Waypoint_evolved(3,1,1),'FaceColor',color_waypoint,'LineStyle','none');
    end

    %Red Endpoint
    Waypoint_evolved = evolve_waypoints(Waypoints(:,:,end),T_animation);
    HandleArray{size(HandleArray,2)+1} = surf(x_sphere+Waypoint_evolved(1,1,1),y_sphere+Waypoint_evolved(2,1,1),z_sphere+Waypoint_evolved(3,1,1),'FaceColor',color_endpoint,'LineStyle','none');

    %Current Time
    %p = min(max(State_start(1:3,1),axis_limits(:,1)),axis_limits(:,2));
    p = axislimits_3D(:,1);
    if (b_draw_text(5) == true)
        HandleArray{size(HandleArray,2)+1} = text(p(1),p(2),p(3),['t = ',num2str(T_animation,'%.3f'),'s'],'FontSize',size_font,'VerticalAlignment','bottom','HorizontalAlignment','left');
    end


    %Wind
    wind_origin_meas = [P(1,index), P(2,index),P(3,index)-0.5];
    HandleArray{size(HandleArray,2)+1} = line([wind_origin_meas(1) wind_origin_meas(1)+A_global(1)],[wind_origin_meas(2) wind_origin_meas(2)+A_global(2)],[wind_origin_meas(3) wind_origin_meas(3)+A_global(3)],'Color',[0.5,0.0,0.5,1.0],'LineWidth',3);

    drawnow;

    if (b_record_video == true)
        if (index_iteration == 1 && iteration_video == video_min)
            M = getframe(h_figure3D);
            M = repmat(M,floor(video_max-video_min)+1,1);
        else
            M(iteration_video+1) = getframe(h_figure3D);
        end
    end

    if (T_animation >= t_video_stop)
        break;
    end

    if (iteration_video ~= video_max)
        for idx = 1:size(HandleArray,2)
            delete(HandleArray{idx});
        end
    end

end
