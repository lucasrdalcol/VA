%%
clear
close all
clc

%% Exercise 1

clear, close all
clc

% Get coordinates in UA from Google Maps to simulate
coords = [40.629399851064456, -8.658209941800724, 0
          40.6300830522477,   -8.659174103635296, 0
          40.62955665195403,  -8.659921820976392, 0
          40.630837178799126, -8.6615549930615,   0];

% Plot the coordinates we get (longiture, latitude)
geoplot(coords(:, 1), coords(:, 2), '-*')

% Transform the Longitude and Latitude coordinates to local coordinates
% starting at the given origin
origin = [coords(1,1), coords(1,2), coords(1,3)];
[xEast, yNorth, zUp] = latlon2local(coords(:, 1), coords(:, 2), coords(:, 3), origin);

% Create a Driving Scenario
s = drivingScenario('GeoReference', origin);
v = vehicle(s);
waypoints = [xEast yNorth zUp];

for n = 1:3
    waypoints = AddMidWaypoints(waypoints);
end

speed = 15;
smoothTrajectory(v, waypoints, speed);

% =============== Para a IMU ==== (Acelerómetrro e Giroscópio)
mountingLocationIMU = [1 2 3];
mountingAnglesIMU = [0 0 0];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');
% Adiciona o sensor de IMU (ReferenceFrame tem de ser ENU.)
myimu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');

% =============== Para o GPS ====
mountingLocationGPS = [1 2 3];
mountingAnglesGPS = [50 40 30];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame');
% Adiciona o sensor de GPS (ReferenceFrame tem de ser ENU.)
mygps = gpsSensor('ReferenceLocation',origin,'ReferenceFrame','ENU');

% =============== Para o Encoder (Hodometria) ====
myencoder = wheelEncoderAckermann('TrackWidth',v.Width,...
'WheelBase',v.Wheelbase,'SampleRate',1/s.SampleTime);

plot(s, 'Waypoints', 'on', 'RoadCenters', 'on')
while advance(s)
    pause(s.SampleTime)
end

%% Exercise 2

clear, close all
clc

% Get coordinates in UA from Google Maps to simulate
coords = [40.629399851064456, -8.658209941800724, 0
          40.6300830522477,   -8.659174103635296, 0
          40.62955665195403,  -8.659921820976392, 0
          40.630837178799126, -8.6615549930615,   0];

% Plot the coordinates we get (longiture, latitude)
geoplot(coords(:, 1), coords(:, 2), '-*')

% Transform the Longitude and Latitude coordinates to local coordinates
% starting at the given origin
origin = [coords(1,1), coords(1,2), coords(1,3)];
[xEast, yNorth, zUp] = latlon2local(coords(:, 1), coords(:, 2), coords(:, 3), origin);

% Create a Driving Scenario
s = drivingScenario('GeoReference', origin);
v = vehicle(s);
waypoints = [xEast yNorth zUp];

for n = 1:3
    waypoints = AddMidWaypoints(waypoints);
end

speed = 15;
smoothTrajectory(v, waypoints, speed);

% =============== Para a IMU ==== (Acelerómetrro e Giroscópio)
mountingLocationIMU = [1 2 3];
mountingAnglesIMU = [0 0 0];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');
% Adiciona o sensor de IMU (ReferenceFrame tem de ser ENU.)
myimu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');

% =============== Para o GPS ====
mountingLocationGPS = [1 2 3];
mountingAnglesGPS = [50 40 30];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame');
% Adiciona o sensor de GPS (ReferenceFrame tem de ser ENU.)
mygps = gpsSensor('ReferenceLocation',origin,'ReferenceFrame','ENU');

% =============== Para o Encoder (Hodometria) ====
myencoder = wheelEncoderAckermann('TrackWidth',v.Width,...
'WheelBase',v.Wheelbase,'SampleRate',1/s.SampleTime);

% Leituras da IMU.
accel = []; % [ax, ay, az]
gyro = []; % [wx, wy, wz]

% Leituras dos 4 encoders das 4 rodas
ticks = []; % [bl, br, fl, fr] % f/b front-back, r/l right-left

% Leituras do GPS.
gpsPos = []; % [lat, lon, alt]
gpsVel = []; % [vx, vy, vz]

% Taxa do GPS em função da taxa de simulação
simSamplesPerGPS = (1/s.SampleTime)/mygps.SampleRate;
% gps.SampleRate é por defeito 1 s, mas pode ser alterado
% s.SampleTime é por defeito 0.01 s, mas pode ser alterado

idx = 1;

plot(s, 'Waypoints', 'on', 'RoadCenters', 'on')
while advance(s)
    groundTruth = state(v);
    % decompõe a estrutura do ground truth com certas conversões
    posVeh = groundTruth.Position;
    orientVeh = quaternion( fliplr(groundTruth.Orientation), ...
    'eulerd','ZYX', 'frame');
    velVeh = groundTruth.Velocity;
    accVeh = groundTruth.Acceleration;
    angvelVeh = deg2rad(groundTruth.AngularVelocity);
    
    % Converte grandezas de movimento do referencial do veículo para a IMU
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = ...
    transformMotion( mountingLocationIMU,orientVeh2IMU, ...
    posVeh,orientVeh,velVeh,accVeh,angvelVeh);
    % Lê e acumula mais uma entrada de valores de acelerações e giroscópio
    [accel(end+1,:), gyro(end+1,:)]= myimu(accIMU,angvelIMU,orientIMU);
    
    % Lê e acumula mais um valor de ticks dos encoders
    ticks(end+1,:) = myencoder(velVeh, angvelVeh, orientVeh);

    % Converte grandezas de movimento do referencial do veículo para o GPS
    [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = ...
    transformMotion(mountingLocationGPS, ...
    orientVeh2GPS,posVeh,orientVeh,velVeh,accVeh,angvelVeh);

    % Lê e acumula mais uma entrada de valores de coordenadas velocidade
    [gpsPos(end+1,:), gpsVel(end+1,:)] = mygps(posGPS,velGPS);

    if ( mod(idx, simSamplesPerGPS) == 0)
        [gpsPos(end+1,:), gpsVel(end+1,:)] = mygps(posGPS,velGPS);
    end
    idx = idx + 1; % idx foi inicializado a 0 antes de iniciar o while

    pause(s.SampleTime)

end

% Plot vehicle position waypoints, ticks, accelerometer, gyroscope, GPS
% Position, GPS Velocity

figure(3)
subplot(2,3,1)
plot(waypoints(:, 1), waypoints(:, 2), '-o')
ylabel('Y (m)')
xlabel('X (m)')
axis equal
grid on
title('Vehicle Position Waypoints')

subplot(2,3,2)
plot(ticks)
ylabel('Wheel Ticks')
xlabel('Time (ms)')
title('Wheel Encoder')

subplot(2,3,3)
plot(accel)
ylabel('Aceleration (m/s2)')
xlabel('Time (ms)')
title('Accelerometer')

subplot(2,3,4)
plot(gyro)
ylabel('Angular Velocity (rad/s)')
xlabel('Time (ms)')
title('Gyroscope')

subplot(2,3,5)
geoplot(gpsPos(:, 1), gpsPos(:, 2))
title('GPS Position')

subplot(2,3,6)
plot(gpsVel)
ylabel('Velocity (m/s2)')
xlabel('Time (s)')
title('GPS Velocity')

%% Exercise 3

clear, close all
clc

% Get coordinates in UA from Google Maps to simulate
coords = [40.629399851064456, -8.658209941800724, 0
          40.6300830522477,   -8.659174103635296, 0
          40.62955665195403,  -8.659921820976392, 0
          40.630837178799126, -8.6615549930615,   0];

% Plot the coordinates we get (longiture, latitude)
geoplot(coords(:, 1), coords(:, 2), '-*')

% Transform the Longitude and Latitude coordinates to local coordinates
% starting at the given origin
origin = [coords(1,1), coords(1,2), coords(1,3)];
[xEast, yNorth, zUp] = latlon2local(coords(:, 1), coords(:, 2), coords(:, 3), origin);

% Create a Driving Scenario
s = drivingScenario('GeoReference', origin);
v = vehicle(s);
waypoints = [xEast yNorth zUp];

for n = 1:3
    waypoints = AddMidWaypoints(waypoints);
end

speed = 15;
smoothTrajectory(v, waypoints, speed);

% =============== Para a IMU ==== (Acelerómetrro e Giroscópio)
mountingLocationIMU = [1 2 3];
mountingAnglesIMU = [0 0 0];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');
% Adiciona o sensor de IMU (ReferenceFrame tem de ser ENU.)
myimu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');

% =============== Para o GPS ====
mountingLocationGPS = [1 2 3];
mountingAnglesGPS = [50 40 30];
% Converte a orientação de Euler para quaternion para o simulador.
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame');
% Adiciona o sensor de GPS (ReferenceFrame tem de ser ENU.)
mygps = gpsSensor('ReferenceLocation',origin,'ReferenceFrame','ENU');

% =============== Para o Encoder (Hodometria) ====
myencoder = wheelEncoderAckermann('TrackWidth',v.Width,...
'WheelBase',v.Wheelbase,'SampleRate',1/s.SampleTime);

% Leituras da IMU.
accel = []; % [ax, ay, az]
gyro = []; % [wx, wy, wz]

% Leituras dos 4 encoders das 4 rodas
ticks = []; % [bl, br, fl, fr] % f/b front-back, r/l right-left

% Leituras do GPS.
gpsPos = []; % [lat, lon, alt]
gpsVel = []; % [vx, vy, vz]

% Taxa do GPS em função da taxa de simulação
simSamplesPerGPS = (1/s.SampleTime)/mygps.SampleRate;
% gps.SampleRate é por defeito 1 s, mas pode ser alterado
% s.SampleTime é por defeito 0.01 s, mas pode ser alterado

idx = 1;

plot(s, 'Waypoints', 'on', 'RoadCenters', 'on')
while advance(s)
    groundTruth = state(v);
    % decompõe a estrutura do ground truth com certas conversões
    posVeh = groundTruth.Position;
    orientVeh = quaternion( fliplr(groundTruth.Orientation), ...
    'eulerd','ZYX', 'frame');
    velVeh = groundTruth.Velocity;
    accVeh = groundTruth.Acceleration;
    angvelVeh = deg2rad(groundTruth.AngularVelocity);
    
    % Converte grandezas de movimento do referencial do veículo para a IMU
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = ...
    transformMotion( mountingLocationIMU,orientVeh2IMU, ...
    posVeh,orientVeh,velVeh,accVeh,angvelVeh);
    % Lê e acumula mais uma entrada de valores de acelerações e giroscópio
    [accel(end+1,:), gyro(end+1,:)]= myimu(accIMU,angvelIMU,orientIMU);
    
    % Lê e acumula mais um valor de ticks dos encoders
    ticks(end+1,:) = myencoder(velVeh, angvelVeh, orientVeh);

    % Converte grandezas de movimento do referencial do veículo para o GPS
    [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = ...
    transformMotion(mountingLocationGPS, ...
    orientVeh2GPS,posVeh,orientVeh,velVeh,accVeh,angvelVeh);

    % Lê e acumula mais uma entrada de valores de coordenadas velocidade
    [gpsPos(end+1,:), gpsVel(end+1,:)] = mygps(posGPS,velGPS);

    if ( mod(idx, simSamplesPerGPS) == 0)
        [gpsPos(end+1,:), gpsVel(end+1,:)] = mygps(posGPS,velGPS);
    end
    idx = idx + 1; % idx foi inicializado a 0 antes de iniciar o while

    pause(s.SampleTime)

end

% Plot vehicle position waypoints, ticks, accelerometer, gyroscope, GPS
% Position, GPS Velocity

figure(3)
subplot(2,3,1)
plot(waypoints(:, 1), waypoints(:, 2), '-o')
ylabel('Y (m)')
xlabel('X (m)')
axis equal
grid on
title('Vehicle Position Waypoints')

subplot(2,3,2)
plot(ticks)
ylabel('Wheel Ticks')
xlabel('Time (ms)')
title('Wheel Encoder')

subplot(2,3,3)
plot(accel)
ylabel('Aceleration (m/s2)')
xlabel('Time (ms)')
title('Accelerometer')

subplot(2,3,4)
plot(gyro)
ylabel('Angular Velocity (rad/s)')
xlabel('Time (ms)')
title('Gyroscope')

subplot(2,3,5)
geoplot(gpsPos(:, 1), gpsPos(:, 2))
title('GPS Position')

subplot(2,3,6)
plot(gpsVel)
ylabel('Velocity (m/s2)')
xlabel('Time (s)')
title('GPS Velocity')

figure(4)
Xgps = zeros(size(gpsVel, 1) + 1, 1);
Ygps = Xgps;
Zgps = Xgps;

for n = 1:size(gpsVel)
    dS = gpsVel(n, :)*s.SampleTime*simSamplesPerGPS;
    Xgps = Xgps(n) + ds(1);
    Ygps = Ygps(n) + ds(2);
    Zgps = Zgps(n) + ds(3);
end