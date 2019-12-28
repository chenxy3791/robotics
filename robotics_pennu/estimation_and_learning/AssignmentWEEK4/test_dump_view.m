% Robotics: Estimation and Learning 
% WEEK 4
% 
% This script is to help run your algorithm and visualize the result from it.

%% Load data
clear all;
close all;

load test_dump.mat
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not need to use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] M is a 2D array containing the occupancy grid map of the location
%     e.g. map(x,y) is a log odds ratio of occupancy probability at (x,y)

%% %% Set parameters
%% param = {};
%% % 1. Decide map resolution, i.e., the number of grids for 1 meter.
%% param.resol = 25;
%% 
%% % 3. Indicate where you will put the origin in pixels
%% param.origin = [685,572]';
%% 
%% param.init_pose = -init_pose; % chenxy: why take the negative?

%% Plot LIDAR data
lidar_local = [ranges(:,1).*cos(scanAngles) -ranges(:,1).*sin(scanAngles)];


% The lidar result from the staring point. 
figure,
plot(0,0,'rs'); hold on;
plot(lidar_local(:,1),lidar_local(:,2),'.-');
axis equal;
set(gca,'YDir','reverse');
xlabel('x');
ylabel('y');
grid on;
title('Lidar measurement in the body frame');
hold off;

% figure, % NOTE: param.init_pose = [0,0,...]
% plot(param.origin(1),param.origin(2),'rs'); hold on;
% plot((lidar_local(:,1)*param.resol)+param.origin(1),...
%      (lidar_local(:,2)*param.resol)+param.origin(2),'.-'); hold on;
% axis equal;
% set(gca,'YDir','reverse');
% xlabel('x');
% ylabel('y');
% grid on;
% title('Lidar measurement in the body frame, mapped onto dicrete map');
% hold off;

%% Run algorithm
% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% For a quicker test, you may take some hundreds frames as input arguments as
% shown.
%tstart = tic;
%ranges_used = ranges;%(:,1:1500);
%%pose = particleLocalization(ranges_used, scanAngles, M, param);
%%myPose = particleLocalization_xiang(ranges_used, scanAngles, M, param);
%myPose = particleLocalization(ranges_used, scanAngles, M, param);
%telapsed = toc(tstart);
%fprintf(1,'telapsed = %g for %d measurements(except the first one)\n',telapsed, size(ranges_used,2)-1);
%load practice-answer.mat;

%% Plot final solution
% The final grid map:
figure;
imagesc(map); hold on;

[X,Y] =meshgrid([1:size(map,2)],[1:size(map,1)]);
figure; plot3(X,Y,map);
title('log odds of the original map');

figure;
M_remove_null = map(:);
M_remove_null(M_remove_null==0.49) = [];
hist(M_remove_null(:),50);

