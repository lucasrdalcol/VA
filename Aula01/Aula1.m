%% Initialization
clear all
clc

% Aula01 - Introduction

%% Save or load sensor data
% save('sns1.mat', 'sns1');

load('sns1.mat');

%%

t = [sns1.Time]; %array with sample times

% % Get position of the car data
% % Alternative 1: with for cycle
% PP = zeros(numel(sns1),3); %preallocate empty array for speed
% for i = 1:numel(sns1)
%     PP(i,:) = sns1(i).ActorPoses(1).Position; %update line i of PP
% end

% Alternative 2: with arrayfun
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns1, 'UniformOutput', false))';

% Plot graph for position through time
subplot(2,2,1)
plot(t, PP)
grid on
legend('x', 'y', 'z')
title('Actor''s 1 Position')
xlabel('t (s)')
ylabel('Position (m)')

% Get velocity of the car data
VV = cell2mat(arrayfun(@(S) S.ActorPoses(1).Velocity', sns1, 'UniformOutput', false))';

% Plot graph for velocity through time
subplot(2,2,2)
plot(t, VV)
grid on
legend('v_x', 'v_y', 'v_z')
title('Actor''s 1 Velocity')
xlabel('t (s)')
ylabel('Velocity (m/s)')

% Differentiation of Position to get the Velocity
h = 0.01; % Sample of time in seconds
VV_Calculated = diff(PP)/h;

% Plot graph for velocity calculated through time
subplot(2,2,3)
plot(t(1:length(VV_Calculated)-1), VV_Calculated(1:end-1,:))
grid on
legend('vv_x', 'vv_y', 'vv_z')
title('Actor''s 1 Velocity calculated')
xlabel('t (s)')
ylabel('Velocity (m/s)')

% Calculate error between the Velocity given by the sensor and velocity
% calculated
E = VV_Calculated - VV(1:end-1, :);
subplot(2,2,4)
plot(t(1:length(VV_Calculated)-1), E(1:end-1,:))
grid on
legend('vv_x', 'vv_y', 'vv_z')
title('Error between velocities')
xlabel('t (s)')
ylabel('Velocity (m/s)')

%%

% Get object detection from radar

% max_objs = 0;
% for n = 1:numel(sns1)
%     objs = sns1(n).ObjectDetections;
%     max_obj = numel(objs);
%     if max_obj > max_objs
%         max_objs = max_obj;
%     end
% end
% 
% array_objs = 1:max_objs;
% 
% objs_detected.obj1 = zeros(numel(sns1),6);
% objs_detected.obj2 = zeros(numel(sns1),6);
% objs_detected.obj3 = zeros(numel(sns1),6);
% objs_detected.obj4 = zeros(numel(sns1),6);
% 
% fns = fieldnames(objs_detected);
% 
% for n = 1:numel(sns1)
%     objs = sns1(n).ObjectDetections;
%     for i = 1:numel(objs)
%         obj = objs_detected.(fns{i});
%         obj(n,:) = objs{i}.Measurement';
%     end
% end

figure(2)
plot(PP(:,1), PP(:,2), 'LineWidth', 5)
view(-90, 90)
grid on
title('Map')
xlabel('X (m)')
ylabel('Y (m)')