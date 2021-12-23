%% visualiza end-effector through ch101 distance sensing
clc; clear; close all

% serial
port = 'COM3'; n_sensors = 4;
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);

% set figure properties
figure('Name','out-of-plane','Position',[1920/5,1080/6,1080,810])
axis([-80 80, -80 80, -200 20]);
hold on; grid on; axis equal
xlabel('x [mm]'); ylabel('y [mm]');  zlabel('z [mm]');
view(45,0);         
% view(50, 20)

% sensor ring vis
[f,p] = stlread_mod('US_sensor_ring.stl');

T_world_ring = makehgtform('xrotate',pi/2,'yrotate',pi/4);
p_ring = T_world_ring*p;
ring = patch('Faces',f,'Vertices',p_ring(1:3,:)','Facecolor',[0.8 0.8 0.6],'Edgecolor','none');
ring.FaceVertexAlphaData = 1.0;    % Set constant transparency 
ring.FaceAlpha = 'flat' ;          % Interpolate to find face transparency
R = (max(p_ring(1,:)) - min(p_ring(1,:)))/2;
geometry.x = [mean(p_ring(1,:)),mean(p_ring(1,:))-R,mean(p_ring(1,:)),mean(p_ring(1,:))+R];
geometry.y = [mean(p_ring(2,:))-R,mean(p_ring(2,:)),mean(p_ring(2,:))+R,mean(p_ring(2,:))];

% sensor 0 measure
l0 = line([geometry.x(1),geometry.x(1)],[geometry.y(1),geometry.y(1)],[0,0],'Color','red','LineWidth',1.5);
% sensor 1 measure
l1 = line([geometry.x(2),geometry.x(2)],[geometry.y(2),geometry.y(2)],[0,0],'Color','red','LineWidth',1.5);
% sensor 2 measure
l2 = line([geometry.x(3),geometry.x(3)],[geometry.y(3),geometry.y(3)],[0,0],'Color','red','LineWidth',1.5);
% sensor 3 measure
l3 = line([geometry.x(4),geometry.x(4)],[geometry.y(4),geometry.y(4)],[0,0],'Color','red','LineWidth',1.5);

% real-time vis
vis_range_flag = [1,0,1,0];  % 0 1 2 3
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

dist = zeros(1,n_sensors); dist_old = dist;
port_flag = zeros(1,n_sensors,'logical');
while 1
    latest = char(readline(sensors));
    % check which port
    for i=1:n_sensors
        port_flag(i) = contains(latest,[num2str(i-1),':']);
    end
    % extract distance
    range_ind = strfind(latest, 'Range: ');
    if ~isempty(range_ind)
        dist(port_flag) = str2double(latest(range_ind+7:range_ind+11));
    else
        dist(port_flag) = nan;
    end
    dist_old = dist;
    fprintf('port0 range: [mm]: %f \n', dist(1))
    fprintf('port1 range: [mm]: %f \n', dist(2))
    fprintf('port2 range: [mm]: %f \n', dist(3))
    fprintf('port3 range: [mm]: %f \n', dist(4))
    
    % show sensor 0 measure
    if ~isnan(dist(1)) && vis_range_flag(1)
        l0.ZData = [0 -dist(1)];
    else
        l0.ZData = [0 0];
    end
    % show sensor 1 measure
    if ~isnan(dist(2))&& vis_range_flag(2)
        l1.ZData = [0 -dist(2)];
    else
        l1.ZData = [0 0];
    end
    % show sensor 2 measure
    if ~isnan(dist(3))&& vis_range_flag(3)
        l2.ZData = [0 -dist(3)];
    else
        l2.ZData = [0 0];
    end
    % show sensor 3 measure
    if ~isnan(dist(4))&& vis_range_flag(4)
        l3.ZData = [0 -dist(4)];
    else
        l3.ZData = [0 0];
    end
    pause(0.0005)
end
 