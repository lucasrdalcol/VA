%% Assignment 1 - Autonomous Vehicles
% Name: Lucas Rodrigues Dal'Col
% Número Mecanográfico: 91352

clear
close all
clc

% Load data from scenario
load('allData_91352.mat', 'allData')

%% Use INS sensor to get position and velocities
t = [allData.Time]; %array with sample times

% Get position of the egocar with arrayfun
PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';

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
VV = cell2mat(arrayfun(@(S) S.ActorPoses(1).Velocity', allData, 'UniformOutput', false))';

% Plot graph for velocity through time
subplot(1,2,2)
plot(t, VV)
grid on
legend('v_x', 'v_y', 'v_z')
title('Ego Vehicle Velocity')
xlabel('t (s)')
ylabel('Velocity (m/s)')

% Calculate the linear distance traveled by the car
Lcar = 0;
for n = 2:size(PP,1)
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    Lcar = Lcar + ds;
end

%% RADAR and Camera Detections

% Plot the car's trajectory map
figure(2)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and all detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on

% Initialize variables
radarDetections = [];
cameraDetections = [];
cameraCar = [];
cameraTruck = [];
cameraBicycle = [];
cameraPedestrian = [];

% Cycle each time sample to get each object detection
for n = 1:numel(allData)
    objs = allData(n).ObjectDetections; % Object detections

    % Calculate the car transformation
    posCar = PP(n,:);
    orCar = [allData(n).ActorPoses(1).Yaw allData(n).ActorPoses(1).Pitch allData(n).ActorPoses(1).Roll] * pi/180;
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
           radarDetections = [radarDetections posWorld(1:2)];

        elseif obj.SensorIndex == 2 || obj.SensorIndex == 4 || obj.SensorIndex == 5 % Cameras Detections
           posObj = obj.Measurement(1:3)';
           orObj = obj.Measurement(4:6)' * pi/180;
           TObjTrans = trvec2tform(posObj);
           TObjRot = eul2tform(orObj);
           TObj = TObjTrans * TObjRot;
           posWorld = TCar * TObj * [0 0 0 1]';
           cameraDetections = [cameraDetections posWorld(1:2)];
           
           % Plot according to the classification
           switch obj.ObjectClassID
               case 1 % Car
                    plot(posWorld(1), posWorld(2), 'go')
                    cameraCar = [cameraCar posWorld(1:2)];
               case 2 % Truck
                    plot(posWorld(1), posWorld(2), 'bo')
                    cameraTruck = [cameraTruck posWorld(1:2)];
               case 3 % Bicycle
                    plot(posWorld(1), posWorld(2), 'co')
                    cameraBicycle = [cameraBicycle posWorld(1:2)];
               case 4 % Pedestrian
                    plot(posWorld(1), posWorld(2), 'mo')
                    cameraPedestrian = [cameraPedestrian posWorld(1:2)];
           end
%            legend("Car's trajectory", 'RADAR Detections', 'Camera Detection - car', 'Camera Detection - Bicycle', 'Camera Detection - pedestrian')
        end
    end
end

radarDetections = radarDetections';
cameraDetections = cameraDetections';
cameraCar = cameraCar';
cameraTruck = cameraTruck';
cameraBicycle = cameraBicycle';
cameraPedestrian = cameraPedestrian';

% Draw detections separately

% For RADAR Detections
figure(3)
subplot(2,3,1)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and RADAR Detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on
plot(radarDetections(:, 1), radarDetections(:, 2), 'ro')
% legend("Car's trajectory", 'RADAR Detections')

% For Camera detections - Cars
subplot(2,2,2)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and Camera Detections - cars")
xlabel('X (m)')
ylabel('Y (m)')
hold on
if ~isempty(cameraCar)
    plot(cameraCar(:, 1), cameraCar(:, 2), 'go')
%     legend("Car's trajectory", 'Camera detections - cars')
end

% For Camera detections - Bicycles
subplot(2,2,3)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and Camera Detections - bicycles")
xlabel('X (m)')
ylabel('Y (m)')
    hold on
if ~isempty(cameraBicycle)
    plot(cameraBicycle(:, 1), cameraBicycle(:, 2), 'co')
%     legend("Car's trajectory", 'Camera detections - bicycles')
end

% For Camera detections - Pedestrians
subplot(2,2,4)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and Camera Detections - pedestrians")
xlabel('X (m)')
ylabel('Y (m)')
hold on
if ~isempty(cameraPedestrian)
    plot(cameraPedestrian(:, 1), cameraPedestrian(:, 2), 'mo')
    %     legend("Car's trajectory", 'Camera detections - pedestrians')
end


%% Count detections stopped cars, pedestrians and bicycles
% Start variables of threshold and minimum number of detections to be a object.
distTreshCar = 9;
distTreshBicycle = 9;
distTreshPedestrian = 5;
numMinDetectionsCars = 15;
numMinDetectionsBicycles = 3;
numMinDetectionsPedestrians = 3;
detectedStopCars = {};
detectedBicyles = {};
detectedPedestrians = {};

% Start counters 
StopCars = 0;
Peds = 0;
Bikes = 0;
InPeds = 0;
countStopCar = true;
countBicycle = true;
countPedestrian = true;

% Sort the detections because we can detect more than two cars at the same
% time for example. So we can join the detections by position.
% https://www.mathworks.com/matlabcentral/answers/526191-reorder-points-and-compare-total-distance?s_tid=srchtitle

% Cars
if ~isempty(cameraCar)
%     [~,idx] = sort(cameraCar(:,1)); % sort just the first column 
%     sorted_cameraCar = cameraCar(idx,:); % reorganize the array of coordinates
    sorted_cameraCar = sortrows(cameraCar,2); % sort Y-values relative to X-values in ascending order
    [~,idu] = unique(sorted_cameraCar(:,2)); % get the index of unique Y-values
    for ii = 2:2:length(idu)-1
        
        sorted_cameraCar(idu(ii-1):idu(ii)-1,1) = sort(sorted_cameraCar(idu(ii-1):idu(ii)-1,1));  % sort X-values 
        % relative to odd indexed unique Y values in ascending order
        sorted_cameraCar(idu(ii):idu(ii+1)-1,1) = sort(sorted_cameraCar(idu(ii):idu(ii+1)-1,1),'descend'); % sort X-values
        % relative to even indexed unique Y values in descending order
    end
end

% Bicycles
if ~isempty(cameraBicycle)
%     [~,idx] = sort(cameraBicycle(:,1));
%     sorted_cameraBicycle = cameraBicycle(idx,:);
    sorted_cameraBicycle = sortrows(cameraBicycle,2); % sort Y-values relative to X-values in ascending order
    [~,idu] = unique(sorted_cameraBicycle(:,2)); % get the index of unique Y-values
    for ii = 2:2:length(idu)-1
        
        sorted_cameraBicycle(idu(ii-1):idu(ii)-1,1) = sort(sorted_cameraBicycle(idu(ii-1):idu(ii)-1,1));  % sort X-values 
        % relative to odd indexed unique Y values in ascending order
        sorted_cameraBicycle(idu(ii):idu(ii+1)-1,1) = sort(sorted_cameraBicycle(idu(ii):idu(ii+1)-1,1),'descend'); % sort X-values
        % relative to even indexed unique Y values in descending order
    end
end

% Pedestrians
if ~isempty(cameraPedestrian)
%     [~,idx] = sort(cameraPedestrian(:,1));
%     sorted_cameraPedestrian = cameraPedestrian(idx,:);
    sorted_cameraPedestrian = sortrows(cameraPedestrian,2); % sort Y-values relative to X-values in ascending order
    [~,idu] = unique(sorted_cameraPedestrian(:,2)); % get the index of unique Y-values
    for ii = 2:2:length(idu)-1
        
        sorted_cameraPedestrian(idu(ii-1):idu(ii)-1,1) = sort(sorted_cameraPedestrian(idu(ii-1):idu(ii)-1,1));  % sort X-values 
        % relative to odd indexed unique Y values in ascending order
        sorted_cameraPedestrian(idu(ii):idu(ii+1)-1,1) = sort(sorted_cameraPedestrian(idu(ii):idu(ii+1)-1,1),'descend'); % sort X-values
        % relative to even indexed unique Y values in descending order
    end
end



% Count the number of stopped cars
if ~isempty(cameraCar)

    % Cycle all car detections
    for n = 1:size(sorted_cameraCar, 1)

        % If the variable to count is true, compute the eucledian distance between
        % that detection and all others
        if countStopCar == true
            ds = sorted_cameraCar - sorted_cameraCar(n, 1:2);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
            
            % Get the indexes of those distances that are below a threshold
            % specified to that object
            idxs = find(ds < distTreshCar);
            
            % The joint detections should be more than a specified minimum
            % number of detections to be classified as a object
            if numel(idxs) >= numMinDetectionsCars
                StopCars = StopCars + 1;
                detectedStopCars{StopCars} = sorted_cameraCar(idxs, :);
                countStopCar = false;
            end

        % After the detections of that object are over, start to compute everything again
        elseif n == idxs(end)
            countStopCar = true;
        end
    end
end

% Same pipeline for Bycicles and Pedestrians, just changing the distance
% threshold for each one.

% Count the number of Bicycles
if ~isempty(cameraBicycle)
    for n = 1:size(sorted_cameraBicycle, 1)
        if countBicycle == true
            ds = sorted_cameraBicycle - sorted_cameraBicycle(n, 1:2);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
        
            idxs = find(ds < distTreshBicycle);
             
            if numel(idxs) >= numMinDetectionsBicycles
                Bikes = Bikes + 1;
                detectedBicyles{Bikes} = sorted_cameraBicycle(idxs, :);
                countBicycle = false;
            end
        elseif n == idxs(end)
            countBicycle = true;
        end
    end
end

% Count the number of pedestrians
if ~isempty(cameraPedestrian)
    for n = 1:size(sorted_cameraPedestrian, 1)
        if countPedestrian == true
            ds = sorted_cameraPedestrian - sorted_cameraPedestrian(n, 1:2);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
        
            idxs = find(ds < distTreshPedestrian);
             
            if numel(idxs) >= numMinDetectionsPedestrians
                Peds = Peds + 1;
                detectedPedestrians{Peds} = sorted_cameraPedestrian(idxs, :);
                countPedestrian = false;
            end
        elseif n == idxs(end)
            countPedestrian = true;
        end
    end
end

%% Detect the number of cars in movement

% Start variables of threshold and minimum number of detections to be a object.
distTreshMovCar = 20;
numMinDetectionsMovCars = 1;

% Start counters 
MovCars = 0;
countMovCars = true;
excludeStopCars = true;

detectedMovCars = {};
movCars_cameraCar = cameraCar;

% Filter the Car Detections to only have the cars in movement
if ~isempty(cameraCar)
    % Cycle all camera Detections
    for i = 1:size(detectedStopCars, 2)
        for n = 1:size(detectedStopCars{i}, 1)
            stopCarDetection = detectedStopCars{i}(n, 1);
            idx = find(stopCarDetection == movCars_cameraCar);
            movCars_cameraCar(idx, :) = [];
        end
    end
end

figure(5)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and Camera Mov Cars detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on
plot(movCars_cameraCar(:, 1), movCars_cameraCar(:, 2), 'ko')

% Count the number of stopped cars
if ~isempty(movCars_cameraCar)

    % Cycle all car detections
    for n = 1:size(movCars_cameraCar, 1)

        % If the variable to count is true, compute the eucledian distance between
        % that detection and all others
        if countMovCars == true
            ds = movCars_cameraCar - movCars_cameraCar(n, 1:2);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
            
            % Get the indexes of those distances that are below a threshold
            % specified to that object
            idxs = find(ds < distTreshMovCar);
            
            % The joint detections should be more than a specified minimum
            % number of detections to be classified as a object
            if numel(idxs) >= numMinDetectionsMovCars
                MovCars = MovCars + 1;
                detectedMovCars{MovCars} = movCars_cameraCar(idxs, :);
                countMovCars = false;
            end

        % After the detections of that object are over, start to compute everything again
        elseif n == idxs(end)
            countMovCars = true;
        end
    end
end


%% Get the linear distance traveled by the ego car when detecting the first objects


% Get the mean of the car and pedestrian detections to represent the object
meanCars = {};
meanPedestrians = {};
firstCar_pos = inf;
firstPedestrian_pos = inf;

% For the first stopped car detected
for n = 1:size(detectedStopCars, 2)
    meanCars{n} = mean(detectedStopCars{n});

    ds = PP(:, 1:2) - meanCars{n};
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);

    [minDs idx_pos] = min(ds);
    
    if idx_pos < firstCar_pos
        firstCar_pos = idx_pos;
    end
end

LStopCar1 = 0;
for n = 2:firstCar_pos
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    LStopCar1 = LStopCar1 + ds;
end

figure(3)
subplot(2,2,2)
plot(PP(firstCar_pos, 1), PP(firstCar_pos, 2), 'r*')


% For the first pedestrian detected
for n = 1:size(detectedPedestrians, 2)
    meanPedestrians{n} = mean(detectedPedestrians{n});

    ds = PP(:, 1:2) - meanPedestrians{n};
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);

    [minDs idx_pos] = min(ds);
    
    if idx_pos < firstPedestrian_pos
        firstPedestrian_pos = idx_pos;
    end
end

Lped1 = 0;
for n = 2:firstPedestrian_pos
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    Lped1 = Lped1 + ds;
end

subplot(2,2,4)
plot(PP(firstPedestrian_pos, 1), PP(firstPedestrian_pos, 2), 'r*')

%% Recognize barriers in RADAR Detections

% Start variables of threshold and minimum number of detections to be a object.
distTreshBarrier = 2.0;
distTreshBicycle = 3.5;
distTreshCar = 4.5;
distTreshPedestrian = 3.0;
numMinDetectionsCars = 2;
numMinDetectionsBicycles = 2;
numMinDetectionsPedestrians = 2;
numMinNoise = 2;

% Start counters 
barriers = 0;
countBarriers = true;
excludeBicycles = true;
excludeCars = true;
excludePedestrians = true;
excludeRadarNoise = true;

barriers_radarDetections = radarDetections;


% Filter the RADAR detections to have only the Barriers
% Filter the car detections
if ~isempty(cameraCar)
    if excludeCars == true
        for n = 1:size(sorted_cameraCar, 1)
            ds = barriers_radarDetections - sorted_cameraCar(n, :);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
    
            idxs = find(ds < distTreshCar);
            
            if numel(idxs) >= numMinDetectionsCars
                barriers_radarDetections(idxs, :) = [];
                excludeCars = false;
            end
        end
    elseif n == idxs(end)
           excludeCars = true;
    end
end

% Filter the bicycle detections
if ~isempty(cameraBicycle)
    if excludeBicycles == true
        for n = 1:size(sorted_cameraBicycle, 1)
            ds = barriers_radarDetections - sorted_cameraBicycle(n, :);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
    
            idxs = find(ds < distTreshBicycle);
            
            if numel(idxs) >= numMinDetectionsBicycles
                barriers_radarDetections(idxs, :) = [];
                excludeBicycles = false;
            end
        end
    elseif n == idxs(end)
           excludeBicycles = true;
    end
end

% Filter the Pedestrians detections
if ~isempty(cameraPedestrian)
    % Cycle all camera Detections
    if excludePedestrians == true
        for n = 1:size(sorted_cameraPedestrian, 1)
            ds = barriers_radarDetections - sorted_cameraPedestrian(n, :);
            ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
    
            idxs = find(ds < distTreshPedestrian);
            
            if numel(idxs) >= numMinDetectionsPedestrians
                barriers_radarDetections(idxs, :) = [];
                excludePedestrians = false;
            end
        end
    elseif n == idxs(end)
           excludePedestrians = true;
    end
end


% Remove noise from RADAR Detections

barriers_NoNoise_radarDetections = barriers_radarDetections;
idxs_noise = [];
if ~isempty(radarDetections)
    % Cycle all barriers RADAR Detections
    for n = 1:size(barriers_radarDetections, 1)
        ds = barriers_radarDetections - barriers_radarDetections(n, :);
        ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);

        idxs = find(ds < distTreshBarrier);
        
        if numel(idxs) <= numMinNoise
            idxs_noise = [idxs_noise, idxs'];
        end
    end
end

barriers_radarDetections(idxs_noise, :) = [];


figure(6)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and RADAR Barrier Detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on
plot(barriers_radarDetections(:, 1), barriers_radarDetections(:, 2), 'ro')

% Discover the beggining of the first barrier to get the linear distance
beginningFirstBarrier_pos = inf;

for n = 1:size(barriers_radarDetections, 1)
    ds = PP(:, 1:2) - barriers_radarDetections(n, :);
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
    
    [minDs idx_pos] = min(ds);
    
    if idx_pos < beginningFirstBarrier_pos
        beginningFirstBarrier_pos = idx_pos;
    end
end


LBarrFirst = 0;
for n = 2:beginningFirstBarrier_pos
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    LBarrFirst = LBarrFirst + ds;
end

plot(PP(beginningFirstBarrier_pos, 1), PP(beginningFirstBarrier_pos, 2), 'g*')


% Discover the ending of the last barrier to get the linear distance
endingLastBarrier_pos = 0;

for n = 1:size(barriers_radarDetections, 1)
    ds = PP(:, 1:2) - barriers_radarDetections(n, :);
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
    
    [minDs idx_pos] = min(ds);
    
    if idx_pos > endingLastBarrier_pos
        endingLastBarrier_pos = idx_pos;
    end
end

LBarrLast = 0;
for n = 2:endingLastBarrier_pos
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    LBarrLast = LBarrLast + ds;
end

plot(PP(endingLastBarrier_pos, 1), PP(endingLastBarrier_pos, 2), 'g*')


%% Save results

results = ['91352', string(Lcar), string(Peds), string(InPeds), string(StopCars), string(MovCars), string(Bikes) ...
    string(Lped1), string(LStopCar1), string(LBarrFirst), string(LBarrLast)];
results = join(results, ',');
file = fopen('TP1_results_91352.txt', 'wt');
fprintf(file, results);
fclose(file);






