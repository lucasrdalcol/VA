function [allData, scenario, sensors] = scenario()
%scenario - Returns sensor detections
%    allData = scenario returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = scenario optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.11 (R2021b) and Automated Driving Toolbox 3.4 (R2021b).
% Generated on: 04-Dec-2021 19:18:19

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    % Get the state of the ego vehicle
    actorState = state(egoVehicle);
    time  = scenario.SimulationTime;

    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    insMeas = {};
    isValidTime = false(1, numSensors);
    isValidLaneTime = false(1, numSensors);
    isValidPointCloudTime = false(1, numSensors);
    isValidINSTime = false(1, numSensors);

    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        % Generate the ego vehicle lane boundaries
        if isa(sensor, 'visionDetectionGenerator')
            maxLaneDetectionRange = min(500,sensor.MaxRange);
            lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        end
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes only')
            [laneDets, ~, isValidTime(sensorIndex)] = sensor(lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes with occlusion')
            [laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'PointCloud')
            if sensor.HasRoadsInputPort
                rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, rdmesh, time);
            else
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, time);
            end
            ptClouds = [ptClouds; ptCloud]; %#ok<AGROW>
        elseif strcmp(type, 'INSMeasurement')
            insMeasCurrent = sensor(actorState, time);
            insMeas = [insMeas; insMeasCurrent]; %#ok<AGROW>
            isValidINSTime(sensorIndex) = true;
        end
    end

    % Aggregate all detections into a structure for later use
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime) || any(isValidINSTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

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
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [3.7 0 0.2], ...
    'RangeLimits', [0 100], ...
    'HasNoise', false, ...
    'DetectionProbability', 1, ...
    'TargetReportFormat', 'Detections', ...
    'HasFalseAlarms', false, ...
    'Profiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [1.9 0], ...
    'HasNoise', false, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Lanes and objects', ...
    'Intrinsics', cameraIntrinsics([1600 1600],[960 540],[1080 1920]), ...
    'ActorProfiles', profiles);
sensors{3} = lidarPointCloudGenerator('SensorIndex', 3, ...
    'SensorLocation', [0.95 0], ...
    'HasNoise', false, ...
    'RangeAccuracy', 1e-06, ...
    'ActorProfiles', profiles);
sensors{4} = insSensor('TimeInput', true, ...
    'RollAccuracy', 0, ...
    'PitchAccuracy', 0, ...
    'YawAccuracy', 0, ...
    'PositionAccuracy', [0 0 0], ...
    'VelocityAccuracy', 0);
numSensors = 4;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [54.8 26.2 0;
    39.8 41.8 0;
    16.4 41.7 0;
    -16.3 36 0;
    -45.5 20.5 0;
    -46.8 -27.6 0;
    -20.2 -44.2 0;
    11.6 -45.5 0;
    40.1 -35.9 0;
    54 -16 0;
    54.8 26.2 0];
marking = [laneMarking('Solid')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the barriers
barrierCenters = [-33.6 35.3 0;
    -41 31.6 0;
    -48.8 25.3 0;
    -55.5 16.2 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'Width', 0.61, ...
    'Height', 0.81, ...
    'Mesh', driving.scenario.jerseyBarrierMesh, 'PlotColor', [0.65 0.65 0.65], 'Name', 'Jersey Barrier1');

barrierCenters = [53.1 -29.7 0;
    56 -24 0;
    59.4 -15.9 0;
    62.6 -2.7 0;
    63.7 7.9 0;
    63.2 16.2 0];
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'Width', 0.61, ...
    'Height', 0.81, ...
    'Mesh', driving.scenario.jerseyBarrierMesh, 'PlotColor', [0.65 0.65 0.65], 'Name', 'Jersey Barrier');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-15.85 -47.05 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [-15.85 -47.05 0.01;
    -0.9 -48.5 0;
    17.1 -46.7 0;
    34.5 -41.8 0;
    43.6 -35.7 0;
    51.9 -24.7 0;
    58.2 -9.9 0;
    60.6 5.7 0;
    59.3 19.4 0;
    55 30.7 0;
    46.1 40.8 0;
    36.1 45.3 0;
    24.4 44.9 0;
    8.8 42.5 0;
    -6 40.3 0;
    -19.7 37.1 0;
    -32.7 32.3 0;
    -42.5 26 0;
    -48.8 19.4 0;
    -53.9 10.3 0;
    -56.4 -1.1 0;
    -55.8 -13.9 0;
    -50.8 -25.5 0;
    -44.1 -33.7 0;
    -32.3 -41.6 0;
    -14.3 -47.5 0];
speed = [20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20;20];
smoothTrajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [21.56 -53.62 0], ...
    'Yaw', 103, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10.42 50.91 0], ...
    'Yaw', -86, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-27.23 42.51 0], ...
    'Yaw', -69, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [12.9 39 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-4.3 32.7 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian1');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [50.2 -12.4 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian2');

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [47.19 34.35 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [47.19 34.35 0.01;
    53.1 25 0;
    56.3 14.6 0;
    56.9 3.8 0;
    54.8 -6.9 0;
    50.9 -17.7 0;
    45.4 -28.3 0;
    38 -35.4 0;
    27.6 -39.9 0;
    18.1 -42.8 0;
    5.5 -43.9 0;
    -4.5 -44.2 0;
    -14.6 -43.6 0;
    -20.1 -42.2 0;
    -27.2 -43.3 0;
    -34.4 -40 0;
    -38.2 -34.1 0;
    -44.7 -28.1 0;
    -48.9 -20.8 0;
    -51.8 -13.8 0;
    -52.7 -4 0;
    -51.4 5.1 0;
    -47.6 14.3 0;
    -40.6 22.9 0;
    -31.1 29 0;
    -19.4 33.2 0;
    -8.5 35.9 0;
    0.6 37.2 0;
    6.7 38.8 0;
    13.1 43.2 0;
    23.8 40.8 0;
    32.7 41.2 0;
    40.8 39.5 0;
    46.8 35.2 0];
speed = [18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18;18];
trajectory(car4, waypoints, speed);

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-29.6 -38.9 0], ...
    'Yaw', 158, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-24.9 27.9 0], ...
    'Yaw', 120, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle1');

actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [2.5 -51.2 0], ...
    'Yaw', 92, ...
    'Mesh', driving.scenario.bicycleMesh, ...
    'Name', 'Bicycle2');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [5.09 -45.93 0.01], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian3');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
elseif isa(sensor, 'insSensor')
    output = 'INSMeasurement';
else
    output = 'Objects only';
end

