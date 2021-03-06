%%
clear
close all
clc

%% Exercise 1

clear
close all
clc

% Load driving scenario
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();

% Default car properties
carLen   = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);

egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Construct Trajectory Generator
connector = trajectoryGeneratorFrenet(refPath);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID);
% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:5)';
[actorID, actorGeom] = obstacleGeometry(capList,actorID);

actorGeom(1).Geometry.Length = carLen*1.5; % in meters
actorGeom(1).Geometry.FixedTransform(1,end) = -actorGeom(1).Geometry.Length*rearAxleRatio; % in meters
actorGeom = repmat(actorGeom(1),5,1);
updateObstacleGeometry(capList,actorID,actorGeom);

% Planning Adaptive Routes Through Traffic
% Define Simulator and Planning Parameters

% Synchronize the simulator's update rate to match the trajectory generator's
% discretization interval.
scenario.SampleTime = connector.TimeResolution; % in seconds

% Define planning parameters.
replanRate = 10; % Hz

% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 0.5:0.5:5;
maxTimeHorizon = max(tSteps); % in seconds

capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);

% Define cost parameters.
latDevWeight    =  1;
timeWeight      = -1;
speedWeight     =  1;

% Reject trajectories that violate the following constraints.
maxAcceleration =  15; % in meters/second^2
maxCurvature    =   1; % 1/meters, or radians/meter
minVelocity     =   0; % in meters/second

% Desired velocity setpoint, used by the cruise control behavior and when
% evaluating the cost of trajectories.
speedLimit = 11; % in meters/second

% Minimum distance the planner should target for following behavior.
safetyGap = 10; % in meters

% % Initialize the simulator and create a chasePlot viewer.
% [scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,egoState,isRunning,lineHandles] = ...
%     exampleHelperInitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

tic;
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;

    % Obter as dete????es
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medi????es para compatibiliza????o e parametriza????o de erros
    tracks = [];

    % Planear a trajet??ria
    currActorState = []; % Ainda n??o possu??mos nenhuma informa????o sobre os outros atores
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList,currActorState, egoState); %??ltimo exercicio da aula anterior
    
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    
    % Mover o veiculo para a posi????o atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end

%% Exercise 2

clear
close all
clc

% Load driving scenario
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();

% Default car properties
carLen   = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);

egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Construct Trajectory Generator
connector = trajectoryGeneratorFrenet(refPath);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID);
% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:5)';
[actorID, actorGeom] = obstacleGeometry(capList,actorID);

actorGeom(1).Geometry.Length = carLen*1.5; % in meters
actorGeom(1).Geometry.FixedTransform(1,end) = -actorGeom(1).Geometry.Length*rearAxleRatio; % in meters
actorGeom = repmat(actorGeom(1),5,1);
updateObstacleGeometry(capList,actorID,actorGeom);

% Planning Adaptive Routes Through Traffic
% Define Simulator and Planning Parameters

% Synchronize the simulator's update rate to match the trajectory generator's
% discretization interval.
scenario.SampleTime = connector.TimeResolution; % in seconds

% Define planning parameters.
replanRate = 10; % Hz

% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 0.5:0.5:5;
maxTimeHorizon = max(tSteps); % in seconds

capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);

% Define cost parameters.
latDevWeight    =  1;
timeWeight      = -1;
speedWeight     =  1;

% Reject trajectories that violate the following constraints.
maxAcceleration =  15; % in meters/second^2
maxCurvature    =   1; % 1/meters, or radians/meter
minVelocity     =   0; % in meters/second

% Desired velocity setpoint, used by the cruise control behavior and when
% evaluating the cost of trajectories.
speedLimit = 11; % in meters/second

% Minimum distance the planner should target for following behavior.
safetyGap = 10; % in meters

% Initialize the tracker
tracker = multiObjectTracker(...
'FilterInitializationFcn', @helperInitPointFilter, ...
'AssignmentThreshold', 30, ...
'ConfirmationThreshold', [4 5], ...
'DeletionThreshold', 3);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

tic;
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;

    % Obter as dete????es
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medi????es para compatibiliza????o e parametriza????o de erros

    tracks = tracker.updateTracks(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end
    
    % Planear a trajet??ria
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPose);

    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList,currActorState, egoState); %??ltimo exercicio da aula anterior
    
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    
    % Mover o veiculo para a posi????o atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);


end

%% Exercise 3

clear
close all
clc

% Load driving scenario
[scenario, egoVehicle, sensors] = createTrackingAndPlanningScenario();

% Default car properties
egoID = 1;
carLen   = scenario.Actors(egoID).Length; % in meters
carWidth = scenario.Actors(egoID).Width; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

% Initial state of the ego vehicle
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50];
refPath = referencePathFrenet(waypoints);

egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

[egoID, egoGeom] = egoGeometry(capList,egoID);
% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% actorID = (1:5)';
% [actorID, actorGeom] = obstacleGeometry(capList,actorID);
% 
% actorGeom(1).Geometry.Length = carLen*1.5; % in meters
% actorGeom(1).Geometry.FixedTransform(1,end) = -actorGeom(1).Geometry.Length*rearAxleRatio; % in meters
% actorGeom = repmat(actorGeom(1),5,1);
% updateObstacleGeometry(capList,actorID,actorGeom);


% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 1:1:5;
maxTimeHorizon = max(tSteps); % in seconds

capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);
% capList.MaxNumSteps = numel(tSteps) + 1;

% Initialize the tracker
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
'AssignmentThreshold',[200 inf],...
'ConfirmationThreshold',[8 10],...
'DeletionThreshold',[5 5]);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

tic;
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;

    % Obter as dete????es
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medi????es para compatibiliza????o e parametriza????o de erros

    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end
    
    % Planear a trajet??ria
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);

    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList,currActorState, egoState); %??ltimo exercicio da aula anterior
    
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    
    % Mover o veiculo para a posi????o atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end

%% Exercise 4

clear
close all
clc

% Load driving scenario
[scenario, egoVehicle, sensors] = P12data();

% Default car properties
egoID = 1;
carLen = scenario.Actors(egoID).Length; % in meters
carWidth = scenario.Actors(egoID).Width; % in meters
rearAxleRatio = 0.25;

% Define road dimensions
laneWidth = carWidth*2; % in meters

% Initial state of the ego vehicle
waypoints = [10.1 0;
             31.4 0;
             39.9 0;
             48.6 0;
             62.5 0;
             73.5 0;
             83.6 0];
refPath = referencePathFrenet(waypoints);

egoState = frenet2global(refPath,[0 0 0 0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

[egoID, egoGeom] = egoGeometry(capList,egoID);
% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% actorID = (1:5)';
% [actorID, actorGeom] = obstacleGeometry(capList,actorID);
% 
% actorGeom(1).Geometry.Length = carLen*1.5; % in meters
% actorGeom(1).Geometry.FixedTransform(1,end) = -actorGeom(1).Geometry.Length*rearAxleRatio; % in meters
% actorGeom = repmat(actorGeom(1),5,1);
% updateObstacleGeometry(capList,actorID,actorGeom);


% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 1:1:5;
maxTimeHorizon = max(tSteps); % in seconds

% capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);
capList.MaxNumSteps = numel(tSteps) + 1;

% Initialize the tracker
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
'AssignmentThreshold',[200 inf],...
'ConfirmationThreshold',[8 10],...
'DeletionThreshold',[5 5]);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

tic;
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;

    % Obter as dete????es
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medi????es para compatibiliza????o e parametriza????o de erros

    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end
    
    % Planear a trajet??ria
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);

    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory_ex4(capList,currActorState, egoState); %??ltimo exercicio da aula anterior
    
    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    
    % Mover o veiculo para a posi????o atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end