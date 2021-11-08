%%
clear
close all
clc

%% Exercise 1

addpath(fullfile(matlabroot, 'toolbox', 'shared', 'tracking', 'fusionlib'));

initialDist = 150; % m
initialSpeed = 50; % km/h
brakeAccel = 3; % m/s^2
finalDist = 1; % m

% a função helperCreateSensorDemoScenario apenas ajuda a parametrizar o
% ensaio e é fornecida pelo toolbox de Automated Driving.
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist);

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

%% Exercise 2

% Create passing scenario
leadDist = 40; % m
speed = 50; % km/h
passSpeed = 70; % km/h
[scenario, egoCar] = helperCreateSensorDemoScenario('Passing', leadDist, speed, passSpeed);

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

%% Exercise 3

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

%% Exercise 4

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

%% Exercise 5

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
    rectangle('Position', [3.7, -0.9, 2*length, width]);
    time = scenario.SimulationTime; % obter o timestamp da simulação
    [dets, ~, isValidTime] = radarSensor(gTruth, time); % obter medições apartir da posição real do sensor no tempo exato da simulação, o output dets contém as medições (detections)
%     if dets < 2*length
%         
%     end
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