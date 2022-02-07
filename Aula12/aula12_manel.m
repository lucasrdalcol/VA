%% Exercise 1
clear, clc, close all
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();
% Collision check time stamps
tSteps = 0.5:0.5:5;

capList = dynamicCapsuleList;
capList.MaxNumSteps = numel(tSteps) + 1;
egoID = 1;
rearAxleRatio = 0.25;
carLen = scenario.Actors(egoID).Length;
carWidth = scenario.Actors(egoID).Width;
laneWidth = carWidth * 2;

[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);
egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento 
    % de medições para compatibilização e parametrização de erros
    tracks = [];
    % Planear a trajetória
    currActorState = []; % Ainda não possuímos nenhuma informação sobre os outros atores
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, ...
        currActorState, egoState); %Último exercicio da aula anterior
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end

%% Exercise 2
clear, clc, close all
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();
% Collision check time stamps
tSteps = 0.5:0.5:5;

capList = dynamicCapsuleList;
capList.MaxNumSteps = numel(tSteps) + 1;
egoID = 1;
rearAxleRatio = 0.25;
carLen = scenario.Actors(egoID).Length;
carWidth = scenario.Actors(egoID).Width;
laneWidth = carWidth * 2;

[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);
egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Defining tracker 
tracker = multiObjectTracker(...
    'FilterInitializationFcn', @helperInitPointFilter, ...
    'AssignmentThreshold', 30, ...
    'ConfirmationThreshold', [4 5], ...
    'DeletionThreshold', 3);

while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento 
    % de medições para compatibilização e parametrização de erros
    tracks = tracker.updateTracks(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end

    % Planear a trajetória
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPose);
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, ...
        currActorState, egoState); %Último exercicio da aula anterior
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end


%% Exercise 3
clear, clc, close all
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();
% Collision check time stamps
tSteps = 1:1:5;

capList = dynamicCapsuleList;
capList.MaxNumSteps = numel(tSteps) + 1;
egoID = 1;
rearAxleRatio = 0.25;
carLen = scenario.Actors(egoID).Length;
carWidth = scenario.Actors(egoID).Width;
laneWidth = carWidth * 2;

[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);
egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Defining tracker 
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
    'AssignmentThreshold',[200 inf],...
    'ConfirmationThreshold',[8 10],...
    'DeletionThreshold',[5 5]);

while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento 
    % de medições para compatibilização e parametrização de erros
    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end

    % Planear a trajetória
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, ...
        currActorState, egoState); %Último exercicio da aula anterior
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end

%% Exercise 4
clear, clc, close all
[scenario, egoVehicle, sensors] = P12data();
% Collision check time stamps
tSteps = 1:1:5;

capList = dynamicCapsuleList;
capList.MaxNumSteps = numel(tSteps) + 1;
egoID = 1;
rearAxleRatio = 0.25;
carLen = scenario.Actors(egoID).Length;
carWidth = scenario.Actors(egoID).Width;
laneWidth = 3.6;

[egoID, egoGeom] = egoGeometry(capList,egoID);

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Initial state of the ego vehicle
waypoints = [10.1 0 0;
    31.4 0 0;
    39.9 0 0;
    48.6 0 0;
    62.5 0 0;
    73.5 0 0;
    83.6 0 0];
waypoints(3, :) = [];
refPath = referencePathFrenet(waypoints);
egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Defining tracker 
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
    'AssignmentThreshold',[200 inf],...
    'ConfirmationThreshold',[8 10],...
    'DeletionThreshold',[5 5]);

while advance(scenario)
    % Current time
    time = scenario.SimulationTime;
    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento 
    % de medições para compatibilização e parametrização de erros
    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end

    % Planear a trajetória
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory_ex4(capList, ...
        currActorState, egoState); %Último exercicio da aula anterior
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end