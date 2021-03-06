%%
clear
close all
clc

% Aula04 - RADAR

%% Exercise 1 - Forward Collision Warning

clear, clc, close all

addpath(fullfile(matlabroot, 'toolbox', 'shared', 'tracking', 'fusionlib'));

% Initialization of the parameters
initialDist = 150; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
finalDist = 1; % m

% a função helperCreateSensorDemoScenario apenas ajuda a parametrizar o
% ensaio e é fornecida pelo toolbox de Automated Driving.
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist);

% Generate the Radar data
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor); % esta função cria uma janela com 3 views diferentes da cena
metrics = struct; % estrutura usada para guardar as métricas de erro
while advance(scenario) % avançar a simulação
    gTruth = targetPoses(egoCar); % obter a localização exata do objeto (ground truth)
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets); % atualizar a visualização
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets); % agregar métricas para análise
    end
end

helperPlotSensorDemoDetections(metrics, 'position', 'reverse range', [-6 6]);
% Desenhar a linha que corresponde à medição expectável compensado a posição do referencial em relação ao pára-choques
tgtCar = scenario.Actors(2);
rearOverhang = tgtCar.RearOverhang;
subplot(1,2,1);
hold on; 
plot(-rearOverhang*[1 1], ylim, 'k'); 
hold off;
legend('Error', '2\sigma noise', 'Rear overhang');

% Distância até ao alvo
radarRange = 30-(radarSensor.MountingLocation(1)+tgtCar.RearOverhang);

% Ângulo ocupado no radar pelo veículo a 30m
width = tgtCar.Width;
azSpan = rad2deg(width/radarRange);

%% Exercise 2 - Measure velocity, Doppler Efect

clear, clc, close all

% Create passing scenario
leadDist = 40; % m
speed = 50; % km/h
passSpeed = 70; % km/h
[scenario, egoCar] = helperCreateSensorDemoScenario('Passing', leadDist, speed, passSpeed);

% Generate the radar
radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Configurar o sensor para medições de velocidade
release(radarSensor);
radarSensor.HasRangeRate = true;
radarSensor.RangeRateResolution = 0.5; % m/s

% Atualizar os profiles para os novos atores
radarSensor.Profiles = actorProfiles(scenario);

restart(scenario)

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor); % esta função cria uma janela com 3 views diferentes da cena
metrics = struct; % estrutura usada para guardar as métricas de erro
while advance(scenario) % avançar a simulação
    gTruth = targetPoses(egoCar); % obter a localização exata do objeto (ground truth)
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets); % atualizar a visualização
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets); % agregar métricas para análise
    end
end

helperPlotSensorDemoDetections(metrics, 'velocity', 'time', [-25 25]);
subplot(1,2,1);
legend('Lead car error', 'Lead car 2\sigma noise', ...
'Pass car error', 'Pass car 2\sigma noise', 'Location', 'northwest');

%% Exercise 3 - Pedestrian detection

clear, clc, close all

% Create passing scenario
initialDist = 150; % m
finalDist = 1; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist, withPedestrian);

radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Configurar o radar
release(radarSensor);
radarSensor.ReferenceRange = 100; % m
radarSensor.ReferenceRCS = 0; % dBsm
radarSensor.DetectionProbability = 0.9;

% Atualizar os profiles para os novos atores
radarSensor.Profiles = actorProfiles(scenario);

restart(scenario)

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor); % esta função cria uma janela com 3 views diferentes da cena
metrics = struct; % estrutura usada para guardar as métricas de erro
while advance(scenario) % avançar a simulação
    gTruth = targetPoses(egoCar); % obter a localização exata do objeto (ground truth)
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets); % atualizar a visualização
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets); % agregar métricas para análise
    end
end

helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%% Exercise 4 - Pedestrian detection with mid-range RADAR

clear, clc, close all

% Create passing scenario
initialDist = 150; % m
finalDist = 1; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, ...
initialSpeed, brakeAccel, finalDist, withPedestrian);

radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [90, 10], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 10, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

% Configurar o radar
release(radarSensor);
radarSensor.ReferenceRange = 50; % m
radarSensor.ReferenceRCS = 0; % dBsm
radarSensor.DetectionProbability = 0.9;

% Atualizar os profiles para os novos atores
radarSensor.Profiles = actorProfiles(scenario);

restart(scenario)

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor); % esta função cria uma janela com 3 views diferentes da cena
metrics = struct; % estrutura usada para guardar as métricas de erro
while advance(scenario) % avançar a simulação
    gTruth = targetPoses(egoCar); % obter a localização exata do objeto (ground truth)
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets); % atualizar a visualização
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets); % agregar métricas para análise
    end
end

helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%% Exercise 5 - Obstacles detection with a warning

clear, clc, close all

addpath(fullfile(matlabroot, 'toolbox', 'shared', 'tracking', 'fusionlib'));

initialDist = 150; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
finalDist = 1; % m

% a função helperCreateSensorDemoScenario apenas ajuda a parametrizar o
% ensaio e é fornecida pelo toolbox de Automated Driving.
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist);

% Pegar comprimento e largura do egoCar para criar bounding box
width = egoCar.Width;
length = egoCar.Length;

radarSensor = drivingRadarDataGenerator(...
'SensorIndex', 1, ...
'TargetReportFormat', 'Detections', ...
'UpdateRate', 10, ...
'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
'FieldOfView', [20, 5], ...
'RangeLimits', [0 150], ...
'AzimuthResolution', 4, ...
'RangeResolution', 2.5, ...
'Profiles', actorProfiles(scenario));

[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor); % esta função cria uma janela com 3 views diferentes da cena
metrics = struct; % estrutura usada para guardar as métricas de erro
while advance(scenario) % avançar a simulação
    gTruth = targetPoses(egoCar); % obter a localização exata do objeto (ground truth)
    position = egoCar.Position;
    rectangle('Position', [egoCar.Wheelbase+egoCar.FrontOverhang, -egoCar.FrontOverhang, 2*length, width]);
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
    % Dets has the measurement  of the detections in egoCar coordinate
    % system

    % Print warning if the object is too close
    if ~isempty(dets)
        if dets{1,1}.Measurement(1) < 2*length + egoCar.Wheelbase+egoCar.FrontOverhang
            sprintf('Warning, impending collision. Distance to the target: %0.2f', dets{1,1}.Measurement(1) - egoCar.Wheelbase - egoCar.FrontOverhang)
        end
    end
    
    
    if isValidTime
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets); % atualizar a visualização
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets); % agregar métricas para análise
    end
end

%% Exercise 6 - Tracking with RADAR

clear, clc, close all

addpath(fullfile(matlabroot, 'examples', 'driving_radar_fusion', 'main'));

% Create the scenario
[scenario, egoVehicle, sensors] = helperCreateMultipathDrivingScenario;

% Load the recorded data
load('MultiPathRadarScenarioRecording.mat','detectionLog','configurationLog');

% Configuration of the sensors from the recording to set up the tracker
[~, sensorConfigurations] = helperAssembleData(detectionLog{1},configurationLog{1});

for i = 1:numel(sensorConfigurations)
    sensorConfigurations{i}.FilterInitializationFcn = @helperInitGGIWFilter;
    sensorConfigurations{i}.SensorTransformFcn = @ctmeas;
end

tracker = trackerPHD('SensorConfigurations', sensorConfigurations,...
'PartitioningFcn',@(x)helperMultipathExamplePartitionFcn(x,2,5),...
'AssignmentThreshold',450,...
'ExtractionThreshold',0.8,...
'ConfirmationThreshold',0.85,...
'MergingThreshold',25,...
'DeletionThreshold',1e-2,...
'BirthRate',1e-2,...
'HasSensorConfigurationsInput',true);

% Create trackGOSPAMetric object to calculate GOSPA metric
gospaMetric = trackGOSPAMetric('Distance','custom', ...
'DistanceFcn',@helperGOSPADistance, ...
'CutoffDistance',35);

% Create display for visualization of results
display = helperMultiPathTrackingDisplay;
% Predicted track list for ghost filtering
predictedTracks = objectTrack.empty(0,1);
% Confusion matrix
confMat = zeros(5,5,numel(detectionLog));
% GOSPA metric
gospa = zeros(4,numel(detectionLog));
% Ground truth
groundTruth = scenario.Actors(2:end);

for i = 1:numel(detectionLog)
    % Advance scene for visualization of ground truth
    advance(scenario);
    % Current time
    time = scenario.SimulationTime;
    % Detections and sensor configurations
    [detections, configurations] = helperAssembleData(detectionLog{i},configurationLog{i});

    % Predict confirmed tracks to current time for classifying ghosts
    if isLocked(tracker)
        predictedTracks = predictTracksToTime(tracker,'confirmed',time);
    end

    % Classify radar detections as targets, ghosts, or static environment
    [targets, ghostStatic, ghostDynamic, static, reflectors, classificationInfo] = helperClassifyRadarDetections(detections, egoVehicle, predictedTracks);
    
    % Pass detections from target and sensor configurations to the tracker
    confirmedTracks = tracker(targets, configurations, time);
    % Visualize the results
    display(egoVehicle, sensors, targets, confirmedTracks, ghostStatic, ghostDynamic, static, reflectors);

    % Calculate GOSPA metric
    [gospa(1, i),~,~,gospa(2,i),gospa(3,i),gospa(4,i)] = gospaMetric(confirmedTracks, groundTruth);

    % Get true classification information and generate confusion matrix
    trueClassificationInfo = helperTrueClassificationInfo(detections);
    confMat(:,:,i) = helperConfusionMatrix(trueClassificationInfo, classificationInfo);
end