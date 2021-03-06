%% TP2 - Condução Autónoma em Ambiente Simulado - VA
% Nome: Lucas Rodrigues Dal'Col
% Número mecanográfico: 91352

clear
close all
clc

% Load driving scenario and sensors
[~, scenario, ~] = TP2_Cenario_1();
[sensors, numSensors] = createSensors(scenario);

egoVehicle = scenario.Actors(1);

scenario.SampleTime = 0.1;

% Default car properties
egoID = 1;
carLen = scenario.Actors(egoID).Length; % in meters
carWidth = scenario.Actors(egoID).Width; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

% Define initial position of the car according to the reference path
refPath = helperGetReferencePath;
egoState = frenet2global(refPath,[0 0 0 -0.5*laneWidth 0 0]);
moveEgoToState(egoVehicle, egoState);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

% Create capsules for the ego car
[egoID, egoGeom] = egoGeometry(capList,egoID);
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 0.5:0.5:5;
maxTimeHorizon = max(tSteps); % in seconds

capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);

% Initialize the tracker
tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
'AssignmentThreshold',[275 inf],...
'ConfirmationThreshold',[10 11],...
'DeletionThreshold',[5 5]);

% Create display for visualizing results
display = TrackingAndPlanningDisplay;

% Run scenario simulation
tic;
while advance(scenario)
    % Current time
    time = scenario.SimulationTime;

    % Obtain detections
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medições para compatibilização e parametrização de erros

    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end
    
    % Plan the trajectory
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);

    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList,currActorState, egoState); %Último exercicio da aula anterior
    
    if ~isempty(optimalTrajectory)
        lastOptimalTrajectory = optimalTrajectory;
    end

    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);
    
    % Move the vehicle to the current position
    egoState = lastOptimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end