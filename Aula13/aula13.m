%%
clear
close all
clc

%% Exercise 1

clear
close all
clc

% Load driving scenario
[allData, scenario, sensors] = TP2_Cenario_1();
egoVehicle = scenario.Actors(1);

% Default car properties
egoID = 1;
carLen = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

% Initial state of the ego vehicle
waypoints = [-43.7   52.9;
              13.7   40.1;
              57.1   14.5;
              42.3  -63;
              115.9 -120.8;
              151   -131.2];
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

% Define the time intervals between current and planned states.
% Collision check time stamps
tSteps = 1:1:5;
maxTimeHorizon = max(tSteps); % in seconds

capList.MaxNumSteps = 1+floor(maxTimeHorizon/scenario.SampleTime);

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

    % Obter as deteções
    detections = helperGenerateDetections(sensors, egoVehicle, time); %Pre-processamento de medições para compatibilização e parametrização de erros

    tracks = tracker(detections, time);

    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = tracker.predictTracksToTime('confirmed',timesteps(i));
    end
    
    % Planear a trajetória
    currActorState = updateCapsuleList(capList, predictedTracks, @stateToPoseFrenet);

    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList,currActorState, egoState); %Último exercicio da aula anterior
    
    % Visualize the results
    display(scenario, egoVehicle, {sensors}, detections, tracks, capList, trajectoryList);
    
    % Mover o veiculo para a posição atual
    egoState = optimalTrajectory(2,:);
    moveEgoToState(egoVehicle,egoState);
end