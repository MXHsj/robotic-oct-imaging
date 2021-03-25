%% online trajectory tracking
clc; clear; close all;
rosshutdown
rosinit('localhost')

% receive new waypoints
reg1_sub = rossubscriber('/reg1_target');
reg2_sub = rossubscriber('/reg2_target');
reg3_sub = rossubscriber('/reg3_target');
reg4_sub = rossubscriber('/reg4_target');
reg1_msg = receive(reg1_sub);
reg2_msg = receive(reg2_sub);
reg3_msg = receive(reg3_sub);
reg4_msg = receive(reg4_sub);

reg1_tar = [reshape(reg1_msg.Data,3,4); 0 0 0 1];
reg2_tar = [reshape(reg2_msg.Data,3,4); 0 0 0 1];
reg3_tar = [reshape(reg3_msg.Data,3,4); 0 0 0 1];
reg4_tar = [reshape(reg4_msg.Data,3,4); 0 0 0 1];
wp = zeros(4,4,4);
wp(:,:,1) = reg1_tar;
wp(:,:,2) = reg2_tar;
wp(:,:,3) = reg3_tar;
wp(:,:,4) = reg4_tar;

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

t1 = 18;
t2 = t1+8;
t3 = t2+8;
t4 = t3+8;
tvec1 = 0:1/freq:t1-1/freq;
tvec2 = t1:1/freq:t2-1/freq;
tvec3 = t2:1/freq:t3-1/freq;
tvec4 = t3:1/freq:t4-1/freq;
tvec = [tvec1, tvec2, tvec3, tvec4];
timePoints = [0, t1, t2, t3, t4];
timeSeg = {tvec1, tvec2, tvec3, tvec4};

disp("generating trajectory ...")
tic;

% interpolate translation
vel_bc = [0 0 0.005 0.005 0;
          0 0 0.005 0.005 0;
          0 0 0.005 0.005 0];
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
while rate.TotalElapsedTime < t4+5
    
%     target_pose(end-2:end) = q(:,curr_step);
%     pos_msg.Data = target_pose;
%     send(pos_pub,pos_msg);
    
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