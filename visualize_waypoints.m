function visualize_waypoints(T)

rotations = tform2quat(T);
translations = tform2trvec(T);
figure
plotTransforms(translations,rotations,'FrameSize',0.008)
grid on
axis equal
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')

% figure
% subplot(2,2,1)
% plot(tvec,reshape(T(1:3,4,:),3,length(tvec)))
% legend('x','y','z')
% title('translation')
% 
% subplot(2,2,2)
% plot(tvec,rotm2eul(T(1:3,1:3,:)))
% legend('yaw','pitch','roll')
% title('rotation')
% 
% subplot(2,2,3)
% plot(tvec,vel)
% legend('wx','wy','wz','vx','vy','vz')
% title('velocity')
% 
% subplot(2,2,4)
% plot(tvec,acc)
% legend('dwx','dwy','dwz','dvx','dvy','dvz')
% title('acceleration')