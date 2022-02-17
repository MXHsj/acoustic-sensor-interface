%% visualize end-effector through VL53L0X distance sensing
clc; clear; close all

% serial
port = 'COM5'; 
IDs = [1 2 3 4];
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);

% set figure properties
figure('Name','sensor ring vis','Position',[1920/5,1080/6,1080,810])
axis([-80 80, -80 80, -250 20]);
hold on; grid on; axis equal
xlabel('x [mm]'); ylabel('y [mm]');  zlabel('z [mm]');
% ============== sensor vis enable ============== 
vis_range_flag = [1,1,1,1];  % 1 2 3 4
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
R = (max(p_ring(1,:)) - min(p_ring(1,:)))/2;
geometry.x = [mean(p_ring(1,:)),mean(p_ring(1,:))-R,mean(p_ring(1,:)),mean(p_ring(1,:))+R];
geometry.y = [mean(p_ring(2,:))-R,mean(p_ring(2,:)),mean(p_ring(2,:))+R,mean(p_ring(2,:))];
% initialize sensor measure visualization
l0 = line([geometry.x(1),geometry.x(1)],[geometry.y(1),geometry.y(1)],[0,0],'Color','red','LineWidth',1.5);
l1 = line([geometry.x(2),geometry.x(2)],[geometry.y(2),geometry.y(2)],[0,0],'Color','red','LineWidth',1.5);
l2 = line([geometry.x(3),geometry.x(3)],[geometry.y(3),geometry.y(3)],[0,0],'Color','red','LineWidth',1.5);
l3 = line([geometry.x(4),geometry.x(4)],[geometry.y(4),geometry.y(4)],[0,0],'Color','red','LineWidth',1.5);
ln = line([geometry.x(1),geometry.x(1)],[geometry.y(2),geometry.y(2)],[-250,-250],'Color','green','LineWidth',1.5);  % normal vector
if vis_range_flag(1)
    text(geometry.x(1),geometry.y(1),15,'port1');
end
if vis_range_flag(2)
    text(geometry.x(2),geometry.y(2),15,'port2');
end
if vis_range_flag(3) 
    text(geometry.x(3),geometry.y(3),15,'port3');
end
if vis_range_flag(4)
    text(geometry.x(4),geometry.y(4),15,'port4');
end

% do real-time visualization
dist = zeros(1,length(IDs));
buffer_size = 5; dist_buffer = [];
port_flag = zeros(1,length(IDs),'logical');
while 1
    latest = char(readline(sensors));
    % check sensor ID
    for i = 1:length(IDs)
        port_flag(i) = contains(latest,[num2str(IDs(i)),':']);
    end
    % extract distance
    range_ind = strfind(latest, 'Range: ');
    if ~isempty(range_ind)
        dist(port_flag) = str2double(latest(range_ind+length('Range: '):range_ind+length('Range: ')+2));
    else
        dist(port_flag) = nan;
    end
    % filtering
    dist_buffer = [dist_buffer; dist];
    if length(dist_buffer) > buffer_size
        dist_buffer(1,:) = [];
    end
    dist_filtered = FilterRawDist(dist_buffer);
    
    % get surface normal
    norm = 120.*GetSurfNorm(dist_filtered); % amplify magnitude  
    tilt = max(min(dot([0,0,-1],norm)/(1*vecnorm(norm)),1),-1);
    fprintf('tilted angle: %f [deg]\n',real(acosd(tilt)))
    % update sensor measure visualization
    ln.XData = [geometry.x(1), geometry.x(1) + norm(1)]; 
    ln.YData = [geometry.y(2), geometry.y(2) + norm(2)]; 
    ln.ZData = [-250, -250 - norm(3)];
    l0.ZData = [0,0]; l1.ZData = [0,0]; l2.ZData = [0,0]; l3.ZData = [0,0];
    if ~isnan(dist_filtered(1)) && vis_range_flag(1)
        l0.ZData = [0, -dist_filtered(1)];
    end
    if ~isnan(dist_filtered(2))&& vis_range_flag(2)
        l1.ZData = [0, -dist_filtered(2)];
    end
    if ~isnan(dist_filtered(3))&& vis_range_flag(3)
        l2.ZData = [0, -dist_filtered(3)];
    end
    if ~isnan(dist_filtered(4))&& vis_range_flag(4)
        l3.ZData = [0, -dist_filtered(4)];
    end
    fprintf('port1: %f[mm]\tport2: %f[mm]\tport3: %f[mm]\tport4: %f[mm]\n',dist(1),dist(2),dist(3),dist(4))
    pause(2*1e-4)
end
 