%% 
clear
clc
close all

%% Exercise 6


veloReader = velodyneFileReader('lidarData_ConstructionRoad.pcap', 'HDL32E');

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

limits = [-vehicleDims.Length/2 vehicleDims.Length/2;
          -vehicleDims.Width/2  vehicleDims.Width/2;
          -vehicleDims.Height   0];

%Aplicar o colormap ao eixo
colormap(lidarViewer.Axes, colorLabels);

minNumPoints = 50;

stopTime = veloReader.StartTime + seconds(30);

while hasFrame(veloReader) && veloReader.CurrentTime < stopTime

    ptCloud = readFrame(veloReader);   
    
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
    
    points.GroundPoints = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta', 10);
    
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
    
    %Atualizar a matriz de índices de cor
    colormapValues(points.ObstaclePoints) = colors.Obstacle;
    

    [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);
    
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);
    
    view(lidarViewer, segmentedPtCloud.Location, labelColorIndex) %Apresentar o plot

end



