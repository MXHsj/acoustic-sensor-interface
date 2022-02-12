%% simulate VL53L0X distance sensing on a flat surface target
clc; clear; close all

% ========== random normal vector generation ==========
vector = GenUnitVector(10, false)';
x_candidate = vector(1,:);
y_candidate = vector(2,:);
z_candidate = vector(3,:);
% x_candidate = [0, 0.0106, -0.0928, 0.1042, 0.0701, -0.0835, 0.1128, -0.1083, 0.0587, -0.0616];
% y_candidate = [0, -0.6497, 0.5638, -0.2503, 0.5883, -0.7060, -0.4262, 0.4348, -0.3523, -0.3385];
% z_candidate = [1, 0.7601, 0.8207, 0.9625, 0.8056, 0.7033, 0.8976, 0.8940, 0.9340, 0.9389];
% =====================================================

%% do simulation
n_sensors = 4;  % 3 to 8

% set figure properties
figure('Name','sensor ring vis','Position',[1920/5,1080/6,1080,810])
axis([-82 82, -82 82, -250 20]);
hold on; grid on; axis equal
xlabel('x [mm]'); ylabel('y [mm]');  zlabel('z [mm]');
view(50, 20)

% sensor ring visualization
[f,p] = stlread_mod('laser_sensor_ring.stl');
T_world_ring = makehgtform('xrotate',pi/2,'yrotate',pi/4);
p_ring = T_world_ring*p;
ring_vis = patch('Faces',f,'Vertices',p_ring(1:3,:)','Facecolor',[0.8 0.8 0.6],'Edgecolor','none');
ring_vis.FaceVertexAlphaData = 1.0;    % Set constant transparency
ring_vis.FaceAlpha = 'flat' ;          % Interpolate to find face transparency
ring.R = ((max(p_ring(1,:)) - min(p_ring(1,:)))/2)*0.75;
ring.L = 125;
ring.origin = [mean(p_ring(1,:)), mean(p_ring(2,:))];
ring.x = zeros(1,n_sensors);
ring.y = zeros(1,n_sensors);
for i = 1:n_sensors
    theta = (i-1)*2*pi/n_sensors;
    ring.x(i) = ring.origin(1)+ring.R*cos(theta);
    ring.y(i) = ring.origin(2)+ring.R*sin(theta);
end

% initialize sensor measurement visualization
l = cell(1,n_sensors);
for i = 1:n_sensors
    l{i} = line([ring.x(i),ring.x(i)],[ring.y(i),ring.y(i)],[0,0],'Color','red','LineWidth',1.5);
    text(ring.x(i),ring.y(i),15,['sensor',num2str(i)]);
end
ln = line([ring.origin(1),ring.origin(1)],[ring.origin(2),ring.origin(2)],[-250,-250],'Color','green','LineWidth',1.5);

% define plane  
plane.normal = [x_candidate(1),y_candidate(1),z_candidate(1)]';
plane.origin = [ring.origin(1),ring.origin(2),-ring.L]';
% plane equation a*x + b*y + c*z + d = 0, where normal=[a,b,c]
d = -dot(plane.origin, plane.normal);
[xx, yy] = meshgrid(ring.origin(1)-1.5*ring.R:10:ring.origin(1)+1.5*ring.R, ... % x range
                    ring.origin(2)-1.5*ring.R:10:ring.origin(2)+1.5*ring.R);    % y range
zz = (-d-plane.normal(1)*xx - plane.normal(2)*yy)*1./plane.normal(3);
plane_vis = surf(xx,yy,zz,'FaceColor','none');
ln_ = line([ring.origin(1),ring.origin(1)],[ring.origin(2),ring.origin(2)],[-250,-250],'Color','black','LineWidth',1.5); % ground truth

%% do real-time visualization
dist = zeros(1,n_sensors);
buffer_size = 10; dist_buffer = []; buffer_enable = false;
same_plane_count = 1; same_plane = 100;
plane_count = 1;
err_rec = [];

% ========== sensor error parameter ==========
p_mean = [0.0000, -0.0000, 0.0054, -0.4773, -5.8807];
p_std = [0.0000, -0.0000, 0.0001, -0.0110, 1.7440];
% ============================================

while plane_count <= 10
    % update plane after every 100 iteration
    if same_plane_count >= same_plane
        plane_count = plane_count + 1;
        if plane_count > length(x_candidate)
            break
        end
        same_plane_count = 1;
        delete(plane_vis);
        plane.normal(3) = z_candidate(plane_count);
        plane.normal(1) = x_candidate(plane_count);
        plane.normal(2) = y_candidate(plane_count);
        d = -dot(plane.origin, plane.normal);
        [xx, yy] = meshgrid(ring.origin(1)-1.5*ring.R:10:ring.origin(1)+1.5*ring.R, ...
                            ring.origin(2)-1.5*ring.R:10:ring.origin(2)+1.5*ring.R);
        zz = (-d-plane.normal(1)*xx - plane.normal(2)*yy)*1./plane.normal(3);
        plane_vis = surf(xx,yy,zz,'FaceColor','none');
    else
        same_plane_count = same_plane_count + 1;
    end
    
    % simulate sensor measurements
    for i = 1:n_sensors
        dist(i) = -(-d-plane.normal(1)*l{i}.XData(1) - plane.normal(2)*l{i}.YData(1))*1./plane.normal(3);
        % generate error
        dist_merr = polyval(p_mean,dist(i));
        dist_err_std = polyval(p_std,dist(i));
        dist(i) = dist(i) + normrnd(dist_merr, dist_err_std);
        % compensate error
        dist(i) = dist(i) - 20;
    end
    
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
    
    % get surface normal & calculate error
    norm = GetNormAtContact(dist_filtered, ring); 
    tilt_est = max(min(dot([0,0,-1]',norm)/(1*vecnorm(norm)),1),-1);
    tilt_gdt = max(min(dot([0,0,1]',plane.normal)/(1*vecnorm(norm)),1),-1);
    fprintf('estimated tilted angle: %f [deg]\t true tilted angle: %f [deg]\n',real(acosd(tilt_est)),real(acosd(tilt_gdt)))
    err_rec = [err_rec, real(acosd(tilt_gdt)) - real(acosd(tilt_est))];

    % update visualization
    vec_amp = 60;  % amplify vector magnitude by 60 times
    ln.XData = [plane.origin(1), plane.origin(1) + vec_amp*norm(1)];
    ln.YData = [plane.origin(2), plane.origin(2) + vec_amp*norm(2)];
    ln.ZData = [plane.origin(3), plane.origin(3) - vec_amp*norm(3)];
    ln_.XData = [plane.origin(1), plane.origin(1) + vec_amp*plane.normal(1)];
    ln_.YData = [plane.origin(2), plane.origin(2) + vec_amp*plane.normal(2)];
    ln_.ZData = [plane.origin(3), plane.origin(3) + vec_amp*plane.normal(3)];
    for i = 1:n_sensors
        l{i}.ZData = [0, 0];
        if ~isnan(dist_filtered(i))
            l{i}.ZData = [0, -dist_filtered(i)];
        end
    end
%     fprintf('port0: %f[mm]\tport1: %f[mm]\tport2: %f[mm]\tport3: %f[mm]\n',dist(1),dist(2),dist(3),dist(4))
    pause(2*1e-2)
end

%% evaluation
figure()
plot(1:length(err_rec),abs(err_rec),'.');
xlabel('sample'); ylabel('absolute error [deg]');
title([num2str(n_sensors),' sensors'])
grid on

%% utilities
function [norm] = GetNormAtContact(dist, ring)

    Pt = [0,0,0];   % [mm] probe tip w.r.t eef frame
    n_sensors = length(dist);
    
    if sum(~isnan(dist)) < 3
        norm = nan(1,3); disp('not enough valid sensor distance')
        return 
    end
    
    % calculate points under eef frame
    P = zeros(n_sensors,3);
    P(:,1) = ring.x - ring.origin(1);
    P(:,2) = ring.y - ring.origin(2);
    P(:,3) = -ring.L + dist;
    P = flipud(P);
    
    % calculate normals
    norms = zeros(n_sensors,3);
    for i = 1:n_sensors
        if i < n_sensors
            norms(i,:) = cross(P(i,:)-Pt,P(i+1,:)-Pt);
        else
            norms(i,:) = cross(P(i,:)-Pt,P(1,:)-Pt);
        end
        if isnan(norms(i,:))
            norms(i,:) = [0,0,0];
        end
    end
%     disp(norms)
    norm = sum(norms);
    norm = normalize(norm,'norm');
    
end