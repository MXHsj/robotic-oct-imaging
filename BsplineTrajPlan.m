%% quintic trajectory plan 
% quintic polynomial interpolation through multiple waypoints
% quaternion interpolation for rotation

clc; clear; close all;

% read data
if ~exist('wp_raw','var')
    load('data/waypoints-03-25-2021(2).mat');
end

if ~exist('home_pose','var')
    load('data/home_pose.mat')
end

% recalculate waypoints
x = reshape(wp_raw(1,4,:),size(wp_raw,3),1)*1000;
y = reshape(wp_raw(2,4,:),size(wp_raw,3),1)*1000;
z = reshape(wp_raw(3,4,:),size(wp_raw,3),1)*1000;

coplane_x = mean(x);
coplane_y = y;
coplane_z = z;
samples = 10;
path = polyfit(coplane_y, coplane_z, 3);
cont_x = coplane_x*ones(samples,1);
cont_y = linspace(y(1),y(end),samples)';
cont_z = polyval(path,cont_y);
% estimate length of the path
dpath = polyder(path);
calc_len = @(x) sqrt(1 + polyval(dpath,x).^2);
path_len = integral(calc_len,min(cont_y),max(cont_y));

%% calculate trajectory
freq = 10;
end_time = 0.25*path_len;
timePoints = linspace(0,end_time,samples);
tInterval = [0 end_time];
tvec = 0:1/freq:end_time;

[s,sd,sdd] = bsplinepolytraj([cont_x, cont_y, cont_z]',tInterval,tvec);

%% visualization
% rotations = tform2quat(T);
% translations = tform2trvec(T);
% figure
% plotTransforms(translations,rotations,'FrameSize',0.008)
% grid on
% xlabel('X [mm]')
% ylabel('Y [mm]')

figure
% plot3(s(1,:),s(2,:),s(3,:),'.b')
plot(s(2,:),s(3,:),'.b')
hold on
plot(y,z,'xr')
plot(cont_y,cont_z,'*g')
grid on
xlabel('y [mm]')
ylabel('z [mm]')
ylim([80 110])
% zlabel('z [mm]')
legend('trajectory','waypoints','fitted points')
title('Bspline trajectory')

% figure
% plot(tvec,sqrt(sd(2,:).^2+sd(3,:).^2))
% xlabel('time [sec]')
% ylabel('velocity [mm/s]')
% title('tangent velocity')

figure
plot(tvec,sd,'LineWidth',1)
ylabel('velocity [mm/s]')
legend('vx','vy','vz');
title('velocity')