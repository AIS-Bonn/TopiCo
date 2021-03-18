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

function [HandleArray] = draw_MAV(P_in,A_in,MAV_d,g,alpha,zoom,index_MAV_bb,size_MAV_sphere,size_MAV_line,size_MAV_line_bb) %codegen

    bb_lines_per_meter = 0.1;
    idx_max = ceil(MAV_d / bb_lines_per_meter);

    HandleArray = zeros(sum(idx_max)*4+4,1);

    color_bb = [0.5 0.5 0.5];

    if (alpha ~= 0.0)

        steps    = 50;
        num_axes = size(P_in,1);

        % Less than 4 axes
        P_in(num_axes+1:4)  = 0;
        A_in(num_axes+1:4)  = 0;
        MAV_d(num_axes+1:4) = 0;

        % MAV
        % scatter3(P_in(1),P_in(2),P_in(3),size_point,'Marker','o','MarkerEdgeColor',[0.0 0.0 0.0],'MarkerFaceColor',[0.0 0.0 0.0],'MarkerFaceAlpha',alpha);
        [x_sphere,y_sphere,z_sphere] = sphere;
        HandleArray(1) = surf(x_sphere*size_MAV_sphere+P_in(1),y_sphere*size_MAV_sphere+P_in(2),z_sphere*size_MAV_sphere+P_in(3),'FaceColor',[0.0 0.0 0.0],'LineStyle','none','FaceAlpha',alpha);

        eul_MAV = [P_in(4), -tan(A_in(1)/g)*zoom, tan(A_in(2)/g)*zoom];
        tform_MAV = affine3d(eul2tform(eul_MAV));
        tform_MAV.T(4,1:3) = P_in(1:3);
        
        [x_cylinder,y_cylinder,z_cylinder] = cylinder(size_MAV_line);
        idx = [1,6,11,16,21];
        x_cylinder = x_cylinder(:,idx);
        y_cylinder = y_cylinder(:,idx);
        z_cylinder = z_cylinder(:,idx);
        
        [x,y,z] = transformPointsForward(tform_MAV,x_cylinder,-z_cylinder*MAV_d(2),y_cylinder);
        HandleArray(2) = mesh(x,y,z,'FaceColor',[1.0 0.0 0.0],'LineStyle','none','FaceAlpha',alpha);

        [x,y,z] = transformPointsForward(tform_MAV,x_cylinder,z_cylinder*MAV_d(2),y_cylinder);
        HandleArray(3) = mesh(x,y,z,'FaceColor',[0.5 0.5 0.5],'LineStyle','none','FaceAlpha',alpha);
      
        [x,y,z] = transformPointsForward(tform_MAV,z_cylinder*MAV_d(1)*2-MAV_d(1),x_cylinder,y_cylinder);
        HandleArray(4) = mesh(x,y,z,'FaceColor',[0.5 0.5 0.5],'LineStyle','none','FaceAlpha',alpha);
        
        % Bounding Box
        if (index_MAV_bb == 2 || index_MAV_bb == 5)
            eul_bb = [P_in(4), 0.0, 0.0];
        elseif (index_MAV_bb == 3 || index_MAV_bb == 6)
            eul_bb = eul_MAV;
        else
            eul_bb = [0.0, 0.0, 0.0]; 
        end
        

        tform_bb = affine3d(eul2tform(eul_bb));
        tform_bb.T(4,1:3) = P_in(1:3);


        if (index_MAV_bb == 1 || index_MAV_bb == 2 || index_MAV_bb == 3)
            
            [x_cylinder,y_cylinder,z_cylinder] = cylinder(size_MAV_line_bb);
            idx = [1,6,11,16,21];
            x_cylinder = x_cylinder(:,idx);
            y_cylinder = y_cylinder(:,idx);
            z_cylinder = z_cylinder(:,idx);
            
            l = 2 * MAV_d(1)/(2*idx_max(1)-1);
            for idx = 0:idx_max(1)-1
                offset = -MAV_d(1) + idx * 2 * l;
                [x,y,z] = transformPointsForward(tform_bb,z_cylinder*l+offset,x_cylinder+MAV_d(2),y_cylinder+MAV_d(3));
                HandleArray(5+0*idx_max(1)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,z_cylinder*l+offset,x_cylinder+MAV_d(2),y_cylinder-MAV_d(3));
                HandleArray(5+1*idx_max(1)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,z_cylinder*l+offset,x_cylinder-MAV_d(2),y_cylinder+MAV_d(3));
                HandleArray(5+2*idx_max(1)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,z_cylinder*l+offset,x_cylinder-MAV_d(2),y_cylinder-MAV_d(3));
                HandleArray(5+3*idx_max(1)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
            end
            
            l = 2 * MAV_d(2)/(2*idx_max(2)-1);
            for idx = 0:idx_max(2)-1
                offset = -MAV_d(2) + idx * 2 * l;
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder+MAV_d(1),z_cylinder*l+offset,y_cylinder+MAV_d(3));
                HandleArray(5+4*idx_max(1)+0*idx_max(2)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder+MAV_d(1),z_cylinder*l+offset,y_cylinder-MAV_d(3));
                HandleArray(5+4*idx_max(1)+1*idx_max(2)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder-MAV_d(1),z_cylinder*l+offset,y_cylinder+MAV_d(3));
                HandleArray(5+4*idx_max(1)+2*idx_max(2)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder-MAV_d(1),z_cylinder*l+offset,y_cylinder-MAV_d(3));
                HandleArray(5+4*idx_max(1)+3*idx_max(2)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
            end

            l = 2 * MAV_d(3)/(2*idx_max(3)-1);
            for idx = 0:idx_max(3)-1
                offset = -MAV_d(3) + idx * 2 * l;
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder+MAV_d(1),y_cylinder+MAV_d(2),z_cylinder*l+offset);
                HandleArray(5+4*idx_max(1)+4*idx_max(2)+0*idx_max(3)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder+MAV_d(1),y_cylinder-MAV_d(2),z_cylinder*l+offset);
                HandleArray(5+4*idx_max(1)+4*idx_max(2)+1*idx_max(3)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder-MAV_d(1),y_cylinder+MAV_d(2),z_cylinder*l+offset);
                HandleArray(5+4*idx_max(1)+4*idx_max(2)+2*idx_max(3)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
                [x,y,z] = transformPointsForward(tform_bb,x_cylinder-MAV_d(1),y_cylinder-MAV_d(2),z_cylinder*l+offset);
                HandleArray(5+4*idx_max(1)+4*idx_max(2)+3*idx_max(3)+idx) = surf(x,y,z,'FaceColor',color_bb,'LineStyle','none','FaceAlpha',alpha);
            end
        elseif (index_MAV_bb == 4 || index_MAV_bb == 5 || index_MAV_bb == 6)
            theta = linspace(0,2*pi,steps);
            P(1,:) = MAV_d(1) * cos(theta);
            P(2,:) = MAV_d(2) * sin(theta);
            P_1(3,:) = MAV_d(3) * ones(steps,1);
            P_2(3,:) = -MAV_d(3) * ones(steps,1);

            [P_1(1,:),P_1(2,:),P_1(3,:)] = transformPointsForward(tform_bb,P(1,:),P(2,:),P_1(3,:));
            [P_2(1,:),P_2(2,:),P_2(3,:)] = transformPointsForward(tform_bb,P(1,:),P(2,:),P_2(3,:));

            HandleArray(5) = plot3(P_1(1,:),P_1(2,:),P_1(3,:),'Color',[0.5 0.5 0.5 alpha],'LineWidth',size_MAV_line_bb,'LineStyle','--');
            HandleArray(6) = plot3(P_2(1,:),P_2(2,:),P_2(3,:),'Color',[0.5 0.5 0.5 alpha],'LineWidth',size_MAV_line_bb,'LineStyle','--');
        end
    end
end



