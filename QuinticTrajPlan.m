%% quintic trajectory plan 
% quintic polynomial interpolation through multiple waypoints
% quaternion interpolation for rotation

clc; clear; close all;

% read data
if ~exist('wp_raw','var')
    load('waypoints-03-23-2021.mat');
end

if ~exist('home_pose','var')
    load('home_pose.mat')
end

% recalculate waypoints
x = reshape(wp_raw(1,4,:),size(wp_raw,3),1)*1000;
y = reshape(wp_raw(2,4,:),size(wp_raw,3),1)*1000;
z = reshape(wp_raw(3,4,:),size(wp_raw,3),1)*1000;

coplane_x = mean(x);
coplane_y = y;
coplane_z = z;
samples = 4;
path = polyfit(coplane_y, coplane_z, 3);
cont_x = coplane_x*ones(samples,1);
cont_y = linspace(y(1),y(end),samples)';
cont_z = polyval(path,cont_y);
% estimate length of the path
dpath = polyder(path);
calc_len = @(x) sqrt(1 + polyval(dpath,x).^2);
path_len = integral(calc_len,min(cont_y),max(cont_y));


wp = wp_raw;
wp(1:3,4,:) = [cont_x, cont_y, cont_z]';

% velocity boundary conditions
lin_vel = 0.05;
x_vel_bc = zeros(samples,1);
y_vel_bc = lin_vel*cos(atan(polyval(dpath,cont_y)));
y_vel_bc(1) = 0; y_vel_bc(end) = 0;
z_vel_bc = lin_vel*sin(atan(polyval(dpath,cont_y)));
z_vel_bc(1) = 0; z_vel_bc(end) = 0;
vel_bc = [x_vel_bc, y_vel_bc, z_vel_bc]';

%% 
freq = 10;
T0 = home_pose;
T1 = wp(:,:,1);
T2 = wp(:,:,2);
T3 = wp(:,:,3);
T4 = wp(:,:,4);
end_time = 0.25*path_len;
timePoints = linspace(0,end_time,size(wp,3));
tInterval = [0 end_time];
tvec = 0:1/freq:end_time;

% [s,sd,sdd] = quinticpolytraj(reshape(wp(1:3,4,:),3,4),timePoints,tvec, ...
%                              'VelocityBoundaryCondition',vel_bc);

[s,sd,sdd] = bsplinepolytraj(reshape(wp(1:3,4,:),3,4),tInterval,tvec);

T = zeros(4,4,length(s));
T(1,1,:) = ones(1,length(s));
T(2,2,:) = ones(1,length(s));
T(3,3,:) = ones(1,length(s));
T(4,4,:) = ones(1,length(s));
T(1:3,4,:) = s;

%% visualization
rotations = tform2quat(T);
translations = tform2trvec(T);
figure
plotTransforms(translations,rotations,'FrameSize',0.008)
grid on
xlabel('X [mm]')
ylabel('Y [mm]')

figure
plot(tvec,s)
legend('x','y','z')
title('translation')

figure
plot(tvec,sd)
legend('vx','vy','vz');
title('velocity')