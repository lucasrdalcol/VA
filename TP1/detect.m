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

radarDetections = [];
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

    partitions = partitionDetections(objs);

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
           radarDetections = [radarDetections posWorld];

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

%% Count detections pedestrians and bicycles
distTreshold = 1;
numMinPoints = 2;

cameraCar_copy1 = cameraCar';
cameraBicycle_copy1 = cameraBicycle';
cameraPedestrian_copy1 = cameraPedestrian';
stopCars = 0;
Peds = 0;
Bikes = 0;

% For cars - it's not working well
for n = 1:size(cameraCar_copy1, 1)
    ds = cameraCar_copy1 - cameraCar_copy1(n, 1:2);
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
 
    idxs = find(ds < distTreshold);
    
    
    if numel(idxs) >= numMinPoints
        cameraCar_copy1(idxs, :) = nan;
        stopCars = stopCars + 1;
    end
end

% For bicycles
for n = 1:size(cameraBicycle_copy1, 1)
    ds = cameraBicycle_copy1 - cameraBicycle_copy1(n, 1:2);
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
 
    idxs = find(ds < distTreshold);
    
    
    if numel(idxs) >= numMinPoints
        cameraBicycle_copy1(idxs, :) = nan;
        Bikes = Bikes + 1;
    end
end

% For Pedestrians
for n = 1:size(cameraPedestrian_copy1, 1)
    ds = cameraPedestrian_copy1 - cameraPedestrian_copy1(n, 1:2);
    ds = sqrt(ds(:, 1).^2 + ds(:, 2).^2);
 
    idxs = find(ds < distTreshold);
    
    
    if numel(idxs) >= numMinPoints
        cameraPedestrian_copy1(idxs, :) = nan;
        Peds = Peds + 1;
    end
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
        
        view(lidarViewer, segmentedPtCloud.Location, labelColorIndex) %Apresentar o plot
    end
end




