%% cubic trajectory plan from scratch (test)

clc; 
clear;
close all;

% read data
if ~exist('wp_raw','var')
    load('waypoints-03-16-2021.mat');
end

% add entry pose and exit pose
start_pose = wp_raw(:,:,1);
start_pose(2,4) = start_pose(2,4) + 0.1;
end_pose = wp_raw(:,:,end);
end_pose(2,4) = end_pose(2,4) - 0.1;
wp = zeros(4,4,size(wp_raw,3)+2);
wp(:,:,1) = start_pose; wp(:,:,end) = end_pose;
wp(:,:,2:end-1) = wp_raw;

x = reshape(wp(1,4,:),size(wp,3),1)*1000;
y = reshape(wp(2,4,:),size(wp,3),1)*1000;
z = reshape(wp(3,4,:),size(wp,3),1)*1000;
YPR = rotm2eul(wp(1:3,1:3,:));
norms = reshape(wp(1:3,3,:),size(wp,3),3)*1000;

% fit cubic polynomial path
coplane_x = mean(x);
coplane_y = y;
coplane_z = z;
samples = 20;
path = polyfit(coplane_y, coplane_z, 4);
cont_y = linspace(y(1),y(end),samples)';
cont_z = polyval(path,cont_y);
% estimate length of the path
dpath = polyder(path);
calc_len = @(x) sqrt(1 + polyval(dpath,x).^2);
path_len = integral(calc_len,min(cont_y),max(cont_y));

% interpolate rotation
roll = interp1(linspace(1,samples,size(wp,3)),YPR(:,end),1:samples)';
pitch = interp1(linspace(1,samples,size(wp,3)),YPR(:,end-1),1:samples)';
yaw = mean(YPR(:,end-2))*ones(1,20)';

% solve cubic polynomial trajectory
Tf = path_len*0.5;  % total time [seconds]
T = linspace(0,Tf,samples)';
lin_vel = 10;   % linear tangantial velocity [mm/s]
ang_vel = 0.01; % angular velocity [rad/s]

A = zeros(2*samples, 6);
A(1:2:end-1,:) = [ones(samples,1),T,T.^2,T.^3,T.^4,T.^5];
A(2:2:end,:) = [zeros(samples,1),ones(samples,1),2*T,3*T.^2,4*T.^3,5*T.^4];

by = zeros(2*samples,1);
by(1:2:end-1) = cont_y;
by(2:2:end) = lin_vel*cos(atan(polyval(dpath,cont_y)));
by(2) = 0; by(end) = 0;
ay = flip(pinv(A)*by);

bz = zeros(2*samples,1);
bz(1:2:end-1) = cont_z;
bz(2:2:end) = lin_vel*sin(atan(polyval(dpath,cont_y)));
bz(2) = 0; bz(end) = 0;
az = flip(pinv(A)*bz);

broll = zeros(2*samples,1);
broll(1:2:end-1) = roll;
broll(4:2:end-2) = ang_vel;
aroll = flip(pinv(A)*broll);

bpitch = zeros(2*samples,1);
bpitch(1:2:end-1) = pitch;
bpitch(4:2:end-2) = ang_vel;
apitch = flip(pinv(A)*bpitch);

% final trajectories
t = 0:0.01:Tf;
y_ref = polyval(ay,t);
dy_ref = polyval(polyder(ay),t);
z_ref = polyval(az,t);
dz_ref = polyval(polyder(az),t);
roll_ref = polyval(aroll,t);
droll_ref = polyval(polyder(aroll),t);
pitch_ref = polyval(apitch,t);
dpitch_ref = polyval(polyder(apitch),t);

%% plot
% figure
% plot3(x,y,z,'.b','MarkerSize',8)
% grid on
% hold on
% axis equal
% plot3(coplane_x*ones(1,samples),cont_y,cont_z,'.g','LineWidth',1)
% 
% xlabel('x [mm]')
% ylabel('y [mm]')
% zlabel('z [mm]')

% axis_length = 70;
% line([0,axis_length],[0,0],[0,0],'Color','red','LineWidth',2)
% line([0,0],[0,axis_length],[0,0],'Color','green','LineWidth',2)
% line([0,0],[0,0],[0,axis_length],'Color','blue','LineWidth',2)


figure
subplot(2,2,1)
plot(t,y_ref,'LineWidth',1)
hold on
plot(t,z_ref,'LineWidth',1)
grid on
xlabel('time [sec]')
ylabel('position [mm]')
legend('y','z')

subplot(2,2,2)
plot(t,dy_ref,'LineWidth',1)
hold on 
plot(t,dz_ref,'LineWidth',1)
grid on
xlabel('time [sec]')
ylabel('linear velocity [mm/s]')
legend('vy','vz')

subplot(2,2,3)
plot(t,roll_ref,'LineWidth',1)
hold on
plot(t,pitch_ref,'LineWidth',1)
grid on
xlabel('time [sec]')
ylabel('orientation [rad]')
legend('roll','pitch')

subplot(2,2,4)
plot(t,droll_ref,'LineWidth',1)
hold on
plot(t,dpitch_ref,'LineWidth',1)
grid on
xlabel('time [sec]')
ylabel('angular velocity [rad/s]')
legend('wx','wy')