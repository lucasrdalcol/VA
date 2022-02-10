function [allData, scenario, sensor] = TP2_Cenario_1()
%TP2_Cenario_1 - Returns sensor detections
%    allData = TP2_Cenario_1 returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = TP2_Cenario_1 optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.11 (R2021b) and Automated Driving Toolbox 3.4 (R2021b).
% Generated on: 27-Jan-2022 01:13:14

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
sensor = createSensors(scenario);

allData = {};

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'UpdateInterval', 0.05, ...
    'SensorLocation', [1.9 0], ...
    'MaxRange', 100, ...
    'MinObjectImageSize', [2 2], ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'UpdateInterval', 0.05, ...
    'SensorLocation', [2.8 0.9], ...
    'Yaw', 90, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
    'UpdateInterval', 0.05, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -90, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'UpdateInterval', 0.05, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1000 1000],[1000 1000],[2000 2000]), ...
    'ActorProfiles', profiles);
numSensors = 4;

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
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [40.14 33.44 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [40.14 33.44 0.01;
    21.7 40.1 0;
    -0.9 45.7 0;
    -18.7 50 0;
    -43.2 55.6 0];
speed = [15;15;15;15;15];
smoothTrajectory(car1, waypoints, speed);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [52.6 -32.4 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [52.6 -32.4 0;
    61.27 -5.8 0.01;
    58.4 16.31 0.01;
    13.14 42.72 0.01;
    -43.8 55.4 0];
speed = [20;20;20;20;20];
smoothTrajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [148.91 -129.13 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [148.91 -129.13 0.01;
    116.2 -118.9 0;
    43.4 -63.4 0;
    58.8 14.6 0;
    14.4 41.9 0;
    -43.6 55.6 0];
speed = [12;12;12;12;12;12];
trajectory(car3, waypoints, speed);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-4.42 36.1 0], ...
    'Yaw', 78, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [45.3 -71.6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [45.3 -71.6 0;
    44.2 -64.3 0;
    59.6 13.2 0;
    14.3 42.2 0;
    -43 55.1 0];
speed = [15;15;15;15;15];
trajectory(car5, waypoints, speed);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [119.89 -130.11 0], ...
    'Yaw', 76, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car6');

