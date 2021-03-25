%% online trajectory tracking
clc; clear; close all;
rosshutdown
rosinit('localhost')

% receive new waypoints
reg1_sub = rossubscriber('/reg1_target');
reg2_sub = rossubscriber('/reg2_target');
reg3_sub = rossubscriber('/reg3_target');
reg4_sub = rossubscriber('/reg4_target');
reg5_sub = rossubscriber('/reg5_target');
reg6_sub = rossubscriber('/reg6_target');
reg7_sub = rossubscriber('/reg7_target');
reg8_sub = rossubscriber('/reg8_target');

reg1_msg = receive(reg1_sub);
reg2_msg = receive(reg2_sub);
reg3_msg = receive(reg3_sub);
reg4_msg = receive(reg4_sub);
reg5_msg = receive(reg5_sub);
reg6_msg = receive(reg6_sub);
reg7_msg = receive(reg7_sub);
reg8_msg = receive(reg8_sub);

reg1_tar = [reshape(reg1_msg.Data,3,4); 0 0 0 1];
reg2_tar = [reshape(reg2_msg.Data,3,4); 0 0 0 1];
reg3_tar = [reshape(reg3_msg.Data,3,4); 0 0 0 1];
reg4_tar = [reshape(reg4_msg.Data,3,4); 0 0 0 1];
reg5_tar = [reshape(reg5_msg.Data,3,4); 0 0 0 1];
reg6_tar = [reshape(reg6_msg.Data,3,4); 0 0 0 1];
reg7_tar = [reshape(reg7_msg.Data,3,4); 0 0 0 1];
reg8_tar = [reshape(reg8_msg.Data,3,4); 0 0 0 1];

wp = zeros(4,4,8);
wp(:,:,1) = reg1_tar;
wp(:,:,2) = reg2_tar;
wp(:,:,3) = reg3_tar;
wp(:,:,4) = reg4_tar;
wp(:,:,5) = reg5_tar;
wp(:,:,6) = reg6_tar;
wp(:,:,7) = reg7_tar;
wp(:,:,8) = reg8_tar;

% velocity boundary conditions
lin_vel = 0.0055;
vel_bc = zeros(3,size(wp,3)+1);
vel_bc(:,3:end-1) = lin_vel*wp(1:3,2,2:end-1);

%% generate trajectory
franka_sub = rossubscriber('/franka_state_custom');
get_franka_state = receive(franka_sub);
current_state = reshape(get_franka_state.Data,4,4)';

freq = 900;
T0 = current_state;

trans_wp = zeros(3,1+size(wp,3));
trans_wp(:,1) = T0(1:3,4);
trans_wp(:,2:end) = reshape(wp(1:3,4,:),3,size(wp,3));

rot_wp = zeros(3,3,1+size(wp,3));
rot_wp(:,:,1) = T0(1:3,1:3);
rot_wp(:,:,2:end) = wp(1:3,1:3,:);

tInterv = 7;
timePoints = zeros(1,size(wp,3)+1);
timePoints(2) = 10;     % home to entry
for i=3:length(timePoints)
    timePoints(i) = timePoints(i-1)+tInterv;
end
timeSeg = cell(1,length(timePoints));
for i = 1:length(timePoints)-1
    timeSeg{i} = timePoints(i):1/freq:timePoints(i+1)-1/freq;
end
tvec = 0:1/freq:timePoints(end)-1/freq;

disp("generating trajectory ...")
tic;
% interpolate translation
[q,dq,ddq] = quinticpolytraj(trans_wp,timePoints,tvec, ...
                             'VelocityBoundaryCondition',vel_bc);
% interpolate rotation
w = [];
for i = 1:size(wp,3)
    [s,ds,dds] = quinticpolytraj([0 1],timePoints(i:i+1),timeSeg{i});
    [~, wi, ~] = rottraj(rot_wp(:,:,i),rot_wp(:,:,i+1), ...
                         timePoints(i:i+1),timeSeg{i}, ...
                         'TimeScaling',[s;ds;dds]);
    w = [w, wi];
end
disp("finish interpolation")
disp(toc)

%% execution
% pos_pub = rospublisher('/target_pose');
% pos_msg = rosmessage(pos_pub);

vel_pub = rospublisher('/cmd_vel');
vel_msg = rosmessage(vel_pub);

rate = rateControl(freq);
curr_step = 1;
cmd_vel = [0,0,0,0,0,0];
while rate.TotalElapsedTime < timePoints(end)+5
    
    cmd_vel(1:3) = dq(:,curr_step);
    cmd_vel(4:6) = w(:,curr_step);
    vel_msg.Data = cmd_vel;
    send(vel_pub,vel_msg);
    
    fprintf("step: %d \n", curr_step);
    if curr_step < length(tvec)
        curr_step = curr_step + 1;
    end
    waitfor(rate);
end
% remove residual motion
vel_msg.Data = [0,0,0,0,0,0];
send(vel_pub,vel_msg);
rosshutdown