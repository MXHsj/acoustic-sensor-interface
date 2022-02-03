%% simulate VL53L0X distance sensing
clc; clear; close all

% set figure properties
n_sensors = 4;
figure('Name','sensor ring vis','Position',[1920/5,1080/6,1080,810])
axis([-82 82, -82 82, -250 20]);
hold on; grid on; axis equal
xlabel('x [mm]'); ylabel('y [mm]');  zlabel('z [mm]');
% ============== sensor vis enable ============== 
% ID: 1, 2, 3, 4
% 1--3 --> out-of-plane view
% 2--4 --> in-plane view
vis_range_flag = [1,1,1,1];
% ===============================================
if sum(vis_range_flag) < 4
    if vis_range_flag(1) && vis_range_flag(3)       % out-of-plane view
        view(45,0); 
    elseif vis_range_flag(2) && vis_range_flag(4)   % in-plane view
        view(-45,0);
    end
else
    view(50, 20)
end

% sensor ring visualization
[f,p] = stlread_mod('laser_sensor_ring.stl');
T_world_ring = makehgtform('xrotate',pi/2,'yrotate',pi/4);
p_ring = T_world_ring*p;
ring = patch('Faces',f,'Vertices',p_ring(1:3,:)','Facecolor',[0.8 0.8 0.6],'Edgecolor','none');
ring.FaceVertexAlphaData = 1.0;    % Set constant transparency
ring.FaceAlpha = 'flat' ;          % Interpolate to find face transparency
R = ((max(p_ring(1,:)) - min(p_ring(1,:)))/2)*0.75;
L = 125;
geometry.x = [mean(p_ring(1,:)),mean(p_ring(1,:))-R,mean(p_ring(1,:)),mean(p_ring(1,:))+R];
geometry.y = [mean(p_ring(2,:))-R,mean(p_ring(2,:)),mean(p_ring(2,:))+R,mean(p_ring(2,:))];

% initialize sensor measure visualization
l1 = line([geometry.x(1),geometry.x(1)],[geometry.y(1),geometry.y(1)],[0,0],'Color','red','LineWidth',1.5);
l2 = line([geometry.x(2),geometry.x(2)],[geometry.y(2),geometry.y(2)],[0,0],'Color','red','LineWidth',1.5);
l3 = line([geometry.x(3),geometry.x(3)],[geometry.y(3),geometry.y(3)],[0,0],'Color','red','LineWidth',1.5);
l4 = line([geometry.x(4),geometry.x(4)],[geometry.y(4),geometry.y(4)],[0,0],'Color','red','LineWidth',1.5);
ln = line([geometry.x(1),geometry.x(1)],[geometry.y(2),geometry.y(2)],[-250,-250],'Color','green','LineWidth',1.5);  % normal vector
if vis_range_flag(1)
    text(geometry.x(1),geometry.y(1),15,'port0');
end
if vis_range_flag(2)
    text(geometry.x(2),geometry.y(2),15,'port1');
end
if vis_range_flag(3) 
    text(geometry.x(3),geometry.y(3),15,'port2');
end
if vis_range_flag(4)
    text(geometry.x(4),geometry.y(4),15,'port3');
end

% define plane  
normal = [0,0,1]';  % ground truth normal vector
origin = [geometry.x(1),geometry.y(2),-150]';
% plane equation a*x + b*y + c*z + d = 0, where normal=[a,b,c]
d = -dot(origin, normal);
[xx, yy] = meshgrid(geometry.x(1)-1.5*R:10:geometry.x(1)+1.5*R, ...
                    geometry.y(2)-1.5*R:10:geometry.y(2)+1.5*R);
zz = (-d-normal(1)*xx - normal(2)*yy)*1./normal(3);
plane = surf(xx,yy,zz,'FaceColor','none');

%% do real-time visualization
IDs = [1, 2, 3, 4];
dist = zeros(1,length(IDs));
buffer_size = 10; dist_buffer = []; buffer_enable = false;
i = 1; n = 100;
while 1
    % simulate sensor distances
    dist(1) = -(-d-normal(1)*l1.XData(1) - normal(2)*l1.YData(1))*1./normal(3);
    dist(2) = -(-d-normal(1)*l2.XData(1) - normal(2)*l2.YData(1))*1./normal(3);
    dist(3) = -(-d-normal(1)*l3.XData(1) - normal(2)*l3.YData(1))*1./normal(3);
    dist(4) = -(-d-normal(1)*l4.XData(1) - normal(2)*l4.YData(1))*1./normal(3);
    dist(1) = dist(1) + normrnd(0, 0.5);
    dist(2) = dist(2) + normrnd(0, 0.5);
    dist(3) = dist(3) + normrnd(0, 0.5);
    dist(4) = dist(4) + normrnd(0, 0.5);
    
    % filtering
    if buffer_enable
        dist_buffer = [dist_buffer; dist];
        if length(dist_buffer) > buffer_size
            dist_buffer(1,:) = [];
        end
        dist_filtered = FilterRawDist(dist_buffer);
    else
        dist_filtered = dist;
    end
    
    % get surface normal
    norm = GetSurfNorm(dist_filtered, R, L); 
    tilt = max(min(dot([0,0,-1],norm)/(1*vecnorm(norm)),1),-1);
%     fprintf('estimated norm [%f,%f,%f]\t tilted angle: %f [deg]\n',norm,real(acosd(tilt)))

    % update sensor measure visualization
    ln.XData = [geometry.x(1), geometry.x(1) + 120*norm(1)]; % amplify magnitude by 120 times
    ln.YData = [geometry.y(2), geometry.y(2) + 120*norm(2)];
    ln.ZData = [-250, -250 - 120*norm(3)];
    l1.ZData = [0,0]; l2.ZData = [0,0]; l3.ZData = [0,0]; l4.ZData = [0,0];
    if ~isnan(dist_filtered(1)) && vis_range_flag(1)
        l1.ZData = [0, -dist_filtered(1)];
    end
    if ~isnan(dist_filtered(2))&& vis_range_flag(2)
        l2.ZData = [0, -dist_filtered(2)];
    end
    if ~isnan(dist_filtered(3))&& vis_range_flag(3)
        l3.ZData = [0, -dist_filtered(3)];
    end
    if ~isnan(dist_filtered(4))&& vis_range_flag(4)
        l4.ZData = [0, -dist_filtered(4)];
    end
%     fprintf('port0: %f[mm]\tport1: %f[mm]\tport2: %f[mm]\tport3: %f[mm]\n',dist(1),dist(2),dist(3),dist(4))
    
    % update plane after 100 iteration
    if i >= n
        delete(plane);
        normal(3) = 0.7+0.3*rand(1);
        normal(1) = -0.15+0.3*rand(1);
        normal(2) = (-1)^randi([1,10],1)*sqrt(1-normal(3)^2-normal(1)^2);
        d = -dot(origin, normal);
        [xx, yy] = meshgrid(geometry.x(1)-1.5*R:10:geometry.x(1)+1.5*R, ...
                            geometry.y(2)-1.5*R:10:geometry.y(2)+1.5*R);
        zz = (-d-normal(1)*xx - normal(2)*yy)*1./normal(3);
        plane = surf(xx,yy,zz,'FaceColor','none');
        i = 1;
    else
        i = i + 1;
    end
    pause(2*1e-2)
end
 