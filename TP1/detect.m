%% Assignment 1 - Autonomous Vehicles

clear 
close all
clc

% Load data from scenario
[allData, scenario, sensors] = scenario();

%% Use INS sensor to get position and velocities
t = [allData.Time]; %array with sample times

% Get position of the egocar with arrayfun
PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))';

% Plot graph for position through time
figure(1)
subplot(1,2,1)
plot(t, PP)
grid on
legend('x', 'y', 'z')
title('Ego Vehicle Position')
xlabel('t (s)')
ylabel('Position (m)')

% Get velocity of the car data
VV = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Velocity', allData, 'UniformOutput', false))';

% Plot graph for velocity through time
subplot(1,2,2)
plot(t, VV)
grid on
legend('v_x', 'v_y', 'v_z')
title('Ego Vehicle Velocity')
xlabel('t (s)')
ylabel('Velocity (m/s)')

Lcar = 0;
for n = 2:size(PP,1)
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    Lcar = Lcar + ds;
end

%% RADAR and Camera Detections

% Plot the car's trajectory map
figure(2)
plot(PP(:,1), PP(:,2), 'LineWidth', 5)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory")
xlabel('X (m)')
ylabel('Y (m)')
hold on

% Cycle each time sample to get each object detection
for n = 1:numel(allData)
    objs = allData(n).ObjectDetections; % Object detections

    % Calculate the car transformation
    posCar = PP(n,:);
    orCar = [fliplr(allData(n).INSMeasurements{1,1}.Orientation)] * pi/180;
    TCarTrans = trvec2tform(posCar);
    TCarRot = eul2tform(orCar);
    TCar = TCarTrans * TCarRot;

    % Cycle each object detection
    for i = 1:numel(objs)
        obj = objs{i, 1};

        % Check the sensor the made the detection and plot the detections
        if obj.SensorIndex == 1 % RADAR Detection
           posObj = obj.Measurement(1:3)';
           orObj = obj.Measurement(4:6)' * pi/180;
           TObjTrans = trvec2tform(posObj);
           TObjRot = eul2tform(orObj);
           TObj = TObjTrans * TObjRot;
           posWorld = TCar * TObj * [0 0 0 1]';
           plot(posWorld(1), posWorld(2), 'ro')

        elseif obj.SensorIndex == 2 % Camera Detection
           posObj = obj.Measurement(1:3)';
           orObj = obj.Measurement(4:6)' * pi/180;
           TObjTrans = trvec2tform(posObj);
           TObjRot = eul2tform(orObj);
           TObj = TObjTrans * TObjRot;
           posWorld = TCar * TObj * [0 0 0 1]';
           
           % Plot according to the classification
           switch obj.ObjectClassID
               case 1 % Car
                    plot(posWorld(1), posWorld(2), 'go')
               case 2 % Truck
                    plot(posWorld(1), posWorld(2), 'bo')
               case 3 % Bicycle
                    plot(posWorld(1), posWorld(2), 'co')
               case 4 % Pedestrian
                    plot(posWorld(1), posWorld(2), 'mo')
           end
%            legend("Car's trajectory", 'RADAR Detections', 'Camera Detection - car', 'Camera Detection - Bicycle', 'Camera Detection - pedestrian')
        end
    end
end



