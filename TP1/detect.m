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
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and all detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on

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
           radarDetections = [radarDetections posWorld(1:2)];

        elseif obj.SensorIndex == 2 % Camera Detection
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


%% Count detections pedestrians and bicycles
% Start variables of threshold and minimum number of detections to be a object.
distTreshCar = 2.0;
distTreshBicycle = 1.0;
distTreshPedestrian = 1.0;
numMinDetections = 4;
detectedStopCars = {};
detectedBicyles = {};
detectedPedestrians = {};

% Start counters 
stopCars = 0;
Peds = 0;
Bikes = 0;
countStopCar = true;
countBicycle = true;
countPedestrian = true;

% Sort the detections because we can detect more than two cars at the same
% time for example. So we can join the detections by position.
% Cars
if ~isempty(cameraCar)
    [~,idx] = sort(cameraCar(:,1)); % sort just the first column 
    sorted_cameraCar = cameraCar(idx,:); % reorganize the array of coordinates
end

% Bicycles
if ~isempty(cameraBicycle)
    [~,idx] = sort(cameraBicycle(:,1));
    sorted_cameraBicycle = cameraBicycle(idx,:);
end

% Pedestrians
if ~isempty(cameraPedestrian)
    [~,idx] = sort(cameraPedestrian(:,1));
    sorted_cameraPedestrian = cameraPedestrian(idx,:);
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
            if numel(idxs) >= numMinDetections
                stopCars = stopCars + 1;
                detectedStopCars{stopCars} = sorted_cameraCar(idxs, :);
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
             
            if numel(idxs) >= numMinDetections
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
             
            if numel(idxs) >= numMinDetections
                Peds = Peds + 1;
                detectedPedestrians{Peds} = sorted_cameraPedestrian(idxs, :);
                countPedestrian = false;
            end
        elseif n == idxs(end)
            countPedestrian = true;
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

%% Recognize barriers in RADAR Detections

% Start variables of threshold and minimum number of detections to be a object.
distTreshBarrier = 20;
distTreshBicycle = 1.5;
distTreshCar = 3;
distTreshPedestrian = 1;
numMinDetections = 4;

% Start counters 
barriers = 0;
countBarriers = true;
excludeBicycles = true;
excludeCars = true;
excludePedestrians = true;

% Sort the detections because we can detect more than two cars at the same
% time for example. So we can join the detections by position.
% RADAR Detections
% if ~isempty(radarDetections)
%     [~,idx] = sort(radarDetections(:,1)); % sort just the first column 
%     sorted_radarDetections = radarDetections(idx,:); % reorganize the array of coordinates
% end

barriers_radarDetections = radarDetections;

% Filter the RADAR detections to have only the Barriers
if ~isempty(cameraCar)
    % Cycle all camera Detections
    if excludeCars == true
        for i = 1:size(detectedStopCars, 2)
            for n = 1:size(detectedStopCars{i}, 1)
                ds = barriers_radarDetections - detectedStopCars{i}(n, :);
                ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
        
                idxs = find(ds < distTreshCar);
                
                if numel(idxs) >= numMinDetections
                    barriers_radarDetections(idxs, :) = [];
                    excludeCars = false;
                end
            end
        end
    elseif n == idxs(end)
           excludeCars = true;
    end
end

if ~isempty(cameraBicycle)
    % Cycle all camera Detections
    if excludeBicycles == true
        for i = 1:size(detectedBicyles, 2)
            for n = 1:size(detectedBicyles{i}, 1)
                ds = barriers_radarDetections - detectedBicyles{i}(n, :);
                ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
        
                idxs = find(ds < distTreshBicycle);
                
                if numel(idxs) >= numMinDetections
                    barriers_radarDetections(idxs, :) = [];
                    excludeBicycles = false;
                end
            end
        end
    elseif n == idxs(end)
           excludeBicycles = true;
    end
end

if ~isempty(cameraPedestrian)
    % Cycle all camera Detections
    if excludePedestrians == true
        for i = 1:size(detectedPedestrians, 2)
            for n = 1:size(detectedPedestrians{i}, 1)
                ds = barriers_radarDetections - detectedPedestrians{i}(n, :);
                ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
        
                idxs = find(ds < distTreshPedestrian);
                
                if numel(idxs) >= numMinDetections
                    barriers_radarDetections(idxs, :) = [];
                    excludePedestrians = false;
                end
            end
        end
    elseif n == idxs(end)
           excludePedestrians = true;
    end
end

figure(5)
plot(PP(:,1), PP(:,2), 'LineWidth', 3)
view(-90, 90)
grid on
axis equal
title("Map of the Car's trajectory and RADAR Barrier Detections")
xlabel('X (m)')
ylabel('Y (m)')
hold on
plot(barriers_radarDetections(:, 1), barriers_radarDetections(:, 2), 'ro')

if ~isempty(barriers_radarDetections)
    % Cycle all camera Detections
    firstBarrier_pos = [barriers_radarDetections(1, :)];
    for n = 2:size(barriers_radarDetections, 1)
        ds = barriers_radarDetections(n, :) - barriers_radarDetections(n-1, :);
        ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);

        if ds < distTreshBarrier
            firstBarrier_pos = [firstBarrier_pos; barriers_radarDetections(n, :)];
        else
            break
        end
    end
end

beginningFirstBarrier = firstBarrier_pos(1, :);
ds = PP(:, 1:2) - beginningFirstBarrier;
ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);

[minDs idx_pos] = min(ds);

LBarrFirst = 0;
for n = 2:idx_pos
    ds = norm(PP(n, 1:2) - PP(n-1, 1:2));
    LBarrFirst = LBarrFirst + ds;
end

%% Lidar detections representation

% Definir os limites da zona a representar
xlimits = [-25 45]; %em metros
ylimits = [-25 45];
zlimits = [-20 20];
lidarViewer = pcplayer(xlimits, ylimits, zlimits); % permite representar um stream de nuvens de pontos 3D

% Definir os labels dos eixos
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');

%Definir um colormap
colorLabels = [0      0.4470 0.7410;
               0.4660 0.6740 0.1880;
               0.9290 0.6940 0.1250;
               0.6350 0.0780 0.1840];

%Indexar as cores
colors.Unlabeled = 1; 
colors.Ground = 2;
colors.Ego = 3;
colors.Obstacle = 4;

vehicleDims = vehicleDimensions(); %4.7m de comprimento, 1.8m de largura, e 1.4m de altura

% Car limits for segmentation
tol = 1;
limits = tol * [-vehicleDims.Length/2 vehicleDims.Length/2;
                -vehicleDims.Width/2  vehicleDims.Width/2;
                -vehicleDims.Height   0];

%Aplicar o colormap ao eixo
colormap(lidarViewer.Axes, colorLabels);

minNumPoints = 50;

% Cycle each time sample to get each object detection
for n = 1:numel(allData)
    
    if ~isempty(allData(n).PointClouds.XLimits)
        ptCloud = allData(n).PointClouds;   
        
        points = struct();
        points.EgoPoints = ptCloud.Location(:,:,1) > limits(1,1) ...
                           & ptCloud.Location(:,:,1) < limits(1,2) ...
                           & ptCloud.Location(:,:,2) > limits(2,1) ...
                           & ptCloud.Location(:,:,2) < limits(2,2) ...
                           & ptCloud.Location(:,:,3) > limits(3,1) ...
                           & ptCloud.Location(:,:,3) < limits(3,2);
        
        scanSize = size(ptCloud.Location);
        scanSize = scanSize(1:2);
        
        %Criar um matriz que indique a cor a usar para cada ponto 32x1084
        colormapValues = ones(scanSize, 'like', ptCloud.Location) * colors.Unlabeled;
        
        %Aplicar a cor aos EgoPoints
        colormapValues(points.EgoPoints) = colors.Ego;
        
        points.GroundPoints = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta', 0.5);
        
        points.GroundPoints = points.GroundPoints & ~points.EgoPoints; %Use only the points that are from the ground.
        % To do this, exclude the points of the car that were already detected.
        
        %Atualizar a matriz de índices de cor
        colormapValues(points.GroundPoints) = colors.Ground;
        
        % Get points without Ego and Ground points
        nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
        
        % Segment original point clouds with nonEgoGroundPoints
        ptCloudSegmented = select(ptCloud, nonEgoGroundPoints, 'Output', 'full');
        
        % Get a mask from origin to a distance of 40 m.
        points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, [0 0 0], 40);
        
        % Segment point cloud for each obstacle 
        [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);
        
        idxValidPoints = find(labels);
        labelColorIndex = labels(idxValidPoints);
        segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);
        
        figure(4)
        view(lidarViewer, segmentedPtCloud.Location, labelColorIndex) %Apresentar o plot
    end
end




