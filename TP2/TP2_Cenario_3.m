function [allData, scenario, sensor] = TP2_Cenario_3()
%TP2_Cenario_3 - Returns sensor detections
%    allData = TP2_Cenario_3 returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = TP2_Cenario_3 optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.11 (R2021b) and Automated Driving Toolbox 3.4 (R2021b).
% Generated on: 09-Feb-2022 10:49:26

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensor(scenario);

allData = {};

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function sensor = createSensor(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensor = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.9 0], ...
    'MaxRange', 100, ...
    'MinObjectImageSize', [2 2], ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 30);

% Add all road segments
roadCenters = [-43.7 52.9 0;
    13.7 40.1 0;
    57.1 14.5 0;
    42.3 -63 0;
    115.9 -120.8 0;
    151 -131.2 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-42.79 50.81 0.01], ...
    'Yaw', -11, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

% Add the non-ego actors
car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [148.91 -129.13 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.494 0.184 0.556], ...
    'Name', 'Car3');
waypoints = [148.91 -129.13 0.01;
    116.2 -118.9 0;
    43.4 -63.4 0;
    58.8 14.6 0;
    14.4 41.9 0;
    -43.6 55.6 0];
speed = [10;10;10;10;10;10];
waittime = [0;0;0;0;0;0];
trajectory(car3, waypoints, speed, waittime);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-4.42 36.1 0], ...
    'Yaw', 78, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.466 0.674 0.188], ...
    'Name', 'Car4');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [119.89 -130.11 0], ...
    'Yaw', 76, ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0.635 0.078 0.184], ...
    'Name', 'Car6');

car7 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-17.5 45.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'PlotColor', [0 0.447 0.741], ...
    'Name', 'Car7');
waypoints = [-17.5 45.2 0;
    13.6 38.2 0;
    55.6 13.5 0;
    40.5 -62.8 0;
    40.3 -69.8 0;
    37.3 -74.5 0];
speed = [3;3;3;3;3;0];
waittime = [0;0;0;0;0;0];
trajectory(car7, waypoints, speed, waittime);
