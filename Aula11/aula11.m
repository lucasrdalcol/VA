%%
clear
close all
clc

openExample('autonomous_control/HighwayTrajectoryPlanningUsingFrenetReferencePathExample')

%% Exercise 1

clear
close all
clc

% Define constants
carLen = 4.7;
carWidth = 1.8;
laneWidth = 3.6;
speedLimit = 15;

% Waypoints of the road
waypoints = [0.2 0.9; 42.5 -5.3; 80.7 -18.4; 112.7 -41.7; 137.4 -75.1; 149.8 -105; 152.2 -142.8];

% Loading scenario created in DSD
scenario = aula11_scenario;

% Reference trajectory from road points
refPath = referencePathFrenet(waypoints);

% Use reference path to create the Frenet Generator
connector = trajectoryGeneratorFrenet(refPath);

scenario.SampleTime = connector.TimeResolution; %Igual o sample time do cenário ao do gerador de trajetórias
replanRate = 1; %Definir o intervalo de tempo entre planeamentos em segundos
stepPerUpdate = (1/replanRate)/scenario.SampleTime; %Definir quantos passos ocorrem entre cada replaneamento

viewer = aula11_scenario; %Criar o Segundo cenário para representação
viewer.SampleTime = scenario.SampleTime;

%Definir um plot do ponto de vista do veículo
chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3,0],'ViewHeight',10,'ViewPitch',20);

egoState = frenet2global(refPath, [0 0 0 laneWidth*3 0 0]); %[S dS ddS L dL ddL] -> [x y theta kappa speed accel]

terminalState = [nan speedLimit 0 laneWidth/2 0 0]; %[S dS ddS L dL ddL], o nan indica
% que a posição não é definida, use o speedLimite = 15 e o laneWidth = 3.6, a posição L ao
% longo da trajetória vai definir para que faixa o veículo se vai deslocar

time = 3; %Tempo total da trajetória a gerar

tic
isRunning = true;

lineHandles = [];
while isRunning
    egoFrenetState = global2frenet(refPath,egoState); %Obter a posição atual do veiculo em coordendas de frenet

    %Criar trajetórias entre o estado atual e o terminal state
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,terminalState,time);
    optimalTrajectory = globalTraj(1).Trajectory; %Selecionar a trajetória
    
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,1,1); %Terá de definir previamente a variável lineHandles = [] (fora do ciclo)

    % Visualize the scene between replanning.
    for i = (2+(0:(stepPerUpdate-1)))
        % Approximate realtime visualization.
        dt = toc;
        if scenario.SampleTime-dt > 0
            pause(scenario.SampleTime-dt);
        end
        egoState = optimalTrajectory(i,:); %Percorrer os passos da trajetória
        %Atualizar o estado do veículo
        viewer.Actors(1).Position(1:2) = egoState(1:2);
        viewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
        viewer.Actors(1).Yaw = egoState(3)*180/pi;
        viewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
        % Update driving scenario.
        isRunning = advance(viewer);
        tic;
    end
end

%% Exercise 2

clear
close all
clc

% Define constants
carLen = 4.7;
carWidth = 1.8;
laneWidth = 3.6;
speedLimit = 15;

% Waypoints of the road
waypoints = [0.2 0.9; 42.5 -5.3; 80.7 -18.4; 112.7 -41.7; 137.4 -75.1; 149.8 -105; 152.2 -142.8];

% Loading scenario created in DSD
scenario = aula11_scenario_ex2;

% Reference trajectory from road points
refPath = referencePathFrenet(waypoints);

% Use reference path to create the Frenet Generator
connector = trajectoryGeneratorFrenet(refPath);

scenario.SampleTime = connector.TimeResolution; %Igual o sample time do cenário ao do gerador de trajetórias
replanRate = 1; %Definir o intervalo de tempo entre planeamentos
stepPerUpdate = (1/replanRate)/scenario.SampleTime; %Definir quantos passos ocorrem entre cada replaneamento

viewer = aula11_scenario_ex2; %Criar o Segundo cenário para representação
viewer.SampleTime = scenario.SampleTime;

%Definir um plot do ponto de vista do veículo
chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3,0],'ViewHeight',10,'ViewPitch',20);

egoState = frenet2global(refPath, [0 0 0 laneWidth*3 0 0]); %[S dS ddS L dL ddL] -> [x y theta kappa speed accel]

terminalState = [nan speedLimit 0 laneWidth/2 0 0]; %[S dS ddS L dL ddL], o nan indica
% que a posição não é definida, use o speedLimite = 15 e o laneWidth = 3.6, a posição L ao
% longo da trajetória vai definir para que faixa o veículo se vai deslocar

time = 3; %Tempo total da trajetória a gerar
maxTimeHorizon = max(time);

% Create Capsule list
capList = dynamicCapsuleList; %Define um objeto que guarda todas as geometrias a utilizar
capList.MaxNumSteps = 20;

% Add ego vehicle to capsule list
egoID = 1;
[egoID, egoGeom] = egoGeometry(capList, egoID); %Criar a geometria do ego veículo

egoGeom.Geometry.Length = carLen; %carLen = 4.7
egoGeom.Geometry.Radius = carWidth/2; %carWidth = 1.8

updateEgoGeometry(capList, egoID, egoGeom); %Atualizar a geometria do ego veículo

% Add other car (actorID 2) to capsule list
actorID = 2;
actorGeom = egoGeom; % Copy egoGeom to actorGeom because the cars have the same dimensions
updateObstacleGeometry(capList, actorID, actorGeom); %Atualizar a geometria dos outros atores

numActors = numel(actorID);
futureTrajectory = repelem(struct('Trajectory',[]), numActors, 1); %Estrutura para guardar as futuras trajetórias dos atores

[actorID, actorPoses] = obstaclePose(capList, capList.ObstacleIDs); %Obter as poses dos atores para usadas para atualizar as capsulas

tic
isRunning = true;

lineHandles = [];
while isRunning
    egoFrenetState = global2frenet(refPath,egoState); %Obter a posição atual do veiculo em coordendas de frenet

    %Criar trajetórias entre o estado atual e o terminal state
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,terminalState,time);
    optimalTrajectory = globalTraj(1).Trajectory; %Selecionar a trajetória

    [curActorState,futureTrajectory,isRunning] = ...
    exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxTimeHorizon); %Obter as futureTrajectories dos vários atores
    
    % Update the collision checker with the predicted trajectories of all actors in the scene.
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);
    
    % Update capsule list with the ego object's candidate trajectory.
    egoPoses.States = optimalTrajectory(:,1:3);
    updateEgoPose(capList,egoID,egoPoses);
    
    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1); %Desenhar as capsulas
    hold off;
    
    isColliding = checkCollision(capList); %Verificar colisões com todos os atores
    
    if all(~isColliding)
        % Nenhuma colisão detetada
    else
        disp('Colisão detetada !!!')
        break;
    end

    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,1,1); %Terá de definir previamente a variável lineHandles = [] (fora do ciclo)
    
    % Visualize the scene between replanning.
    for i = (2+(0:(stepPerUpdate-1)))
        % Approximate realtime visualization.
        dt = toc;
        if scenario.SampleTime-dt > 0
            pause(scenario.SampleTime-dt);
        end
        egoState = optimalTrajectory(i,:); %Percorrer os passos da trajetória
        %Atualizar o estado do veículo
        viewer.Actors(1).Position(1:2) = egoState(1:2);
        viewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
        viewer.Actors(1).Yaw = egoState(3)*180/pi;
        viewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
        % Update driving scenario.
        isRunning = advance(viewer);
        tic;
    end
end


%% Exercise 3a - Copy from the 'Highway Trajectory Planning Using Frenet Reference Path' example from MATLAB

clear
close all
clc

% Load driving scenario
scenario = drivingScenarioTrafficExample;
% Default car properties
carLen   = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

plot(scenario);

% Construct reference path
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50]; % in meters
refPath = referencePathFrenet(waypoints);
ax = show(refPath);
axis(ax,'equal'); xlabel('X'); ylabel('Y');

% Construct Trajectory Generator
connector = trajectoryGeneratorFrenet(refPath);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID);

egoGeom.Geometry.Length = carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; % in meters

updateEgoGeometry(capList,egoID,egoGeom);

actorID = (1:5)';
actorGeom = repelem(egoGeom,5,1);
updateObstacleGeometry(capList,actorID,actorGeom);

% Planning Adaptive Routes Through Traffic
% Define Simulator and Planning Parameters

% Synchronize the simulator's update rate to match the trajectory generator's
% discretization interval.
scenario.SampleTime = connector.TimeResolution; % in seconds

% Define planning parameters.
replanRate = 10; % Hz

% Define the time intervals between current and planned states.
timeHorizons = 1:3; % in seconds
maxHorizon = max(timeHorizons); % in seconds

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

% Initialize Simulator
[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,egoState,isRunning,lineHandles] = ...
    exampleHelperInitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);

% Run Driving Simulation
tic
while isRunning
    % Retrieve the current state of actor vehicles and their trajectory over
    % the planning horizon.
    [curActorState,futureTrajectory,isRunning] = ...
        exampleHelperRetrieveActorGroundTruth(scenario,futureTrajectory,replanRate,maxHorizon);

    % Generate cruise control states.
    [termStatesCC,timesCC] = exampleHelperBasicCruiseControl(...
        refPath,laneWidth,egoState,speedLimit,timeHorizons);
    
    % Generate lane change states.
    [termStatesLC,timesLC] = exampleHelperBasicLaneChange(...
        refPath,laneWidth,egoState,timeHorizons);
    
    % Generate vehicle following states.
    [termStatesF,timesF] = exampleHelperBasicLeadVehicleFollow(...
        refPath,laneWidth,safetyGap,egoState,curActorState,timeHorizons);

    % Combine the terminal states and times.
    allTS = [termStatesCC; termStatesLC; termStatesF];
    allDT = [timesCC; timesLC; timesF];
    numTS = [numel(timesCC); numel(timesLC); numel(timesF)];
    
    % Evaluate cost of all terminal states.
    costTS = exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit,...
        speedWeight, latDevWeight, timeWeight);

    % Generate trajectories.
    egoFrenetState = global2frenet(refPath,egoState);
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);
    
    % Eliminate trajectories that violate constraints.
    isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);

    % Update the collision checker with the predicted trajectories
    % of all actors in the scene.
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList,actorID,actorPoses);
    
    % Determine evaluation order.
    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    
    trajectoryEvaluation = nan(numel(isValid),1);
    
    % Check each trajectory for collisions starting with least cost.
    for i = 1:numel(idx)
        if isValid(idx(i))
            % Update capsule list with the ego object's candidate trajectory.
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            updateEgoPose(capList,egoID,egoPoses);
            
            % Check for collisions.
            isColliding = checkCollision(capList);
            
            if all(~isColliding)
                % If no collisions are found, this is the optimal.
                % trajectory.
                trajectoryEvaluation(idx(i)) = 1;
                optimalTrajectory = globalTraj(idx(i)).Trajectory;
                break;
            else
                trajectoryEvaluation(idx(i)) = 0;
            end
        end
    end

    % Display the sampled trajectories.
    lineHandles = exampleHelperVisualizeScene(lineHandles,globalTraj,isValid,trajectoryEvaluation);
    
    hold on;
    show(capList,'TimeStep',1:capList.MaxNumSteps,'FastUpdate',1);
    hold off;
    
    if isempty(optimalTrajectory)
        % All trajectories either violated a constraint or resulted in collision.
        %
        %   If planning failed immediately, revisit the simulator, planner,
        %   and behavior properties.
        %
        %   If the planner failed midway through a simulation, additional
        %   behaviors can be introduced to handle more complicated planning conditions.
        error('No valid trajectory has been found.');
    else
        % Visualize the scene between replanning.
        for i = (2+(0:(stepPerUpdate-1)))
            % Approximate realtime visualization.
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            
            egoState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = egoState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
            scenarioViewer.Actors(1).Yaw = egoState(3)*180/pi;
            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            
            % Update capsule visualization.
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
            
            % Update driving scenario.
            advance(scenarioViewer);
            tic;
        end
    end
end

%% Exercise 3b - Copy from the 'Highway Trajectory Planning Using Frenet Reference Path' example from MATLAB

clear
close all
clc

% Load driving scenario
scenario = drivingScenarioTrafficExample;
% Default car properties
carLen   = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

plot(scenario);

% Construct reference path
waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50]; % in meters
refPath = referencePathFrenet(waypoints);
ax = show(refPath);
axis(ax,'equal'); xlabel('X'); ylabel('Y');

% Construct Trajectory Generator
connector = trajectoryGeneratorFrenet(refPath);

% Construct Dynamic Collision Checker 
capList = dynamicCapsuleList;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID);

egoGeom.Geometry.Length = carLen; % in meters
egoGeom.Geometry.Radius = laneWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -carLen*rearAxleRatio; % in meters

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
timeHorizons   = 1:5; % in seconds
maxTimeHorizon = max(timeHorizons); % in seconds

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

% Initialize the simulator and create a chasePlot viewer.
[scenarioViewer,futureTrajectory,actorID,actorPoses,egoID,egoPoses,stepPerUpdate,egoState,isRunning,lineHandles] = ...
    exampleHelperInitializeSimulator(scenario,capList,refPath,laneWidth,replanRate,carLen);
tic;
while isRunning
    % Retrieve the current state of actor vehicles and their trajectory over
    % the planning horizon.
    [curActorState,futureTrajectory,isRunning] = exampleHelperRetrieveActorGroundTruth(...
        scenario,futureTrajectory,replanRate,maxTimeHorizon);
    
    % Generate cruise control states.
    [termStatesCC,timesCC] = exampleHelperBasicCruiseControl(...
        refPath,laneWidth,egoState,speedLimit,timeHorizons);
    
    % Generate lane change states.
    [termStatesLC,timesLC] = exampleHelperBasicLaneChange(...
        refPath,laneWidth,egoState,timeHorizons);
    
    % Generate vehicle following states.
    [termStatesF,timesF] = exampleHelperBasicLeadVehicleFollow(...
        refPath,laneWidth,safetyGap,egoState,curActorState,timeHorizons);
    
    % Combine the terminal states and times.
    allTS = [termStatesCC; termStatesLC; termStatesF];
    allDT = [timesCC; timesLC; timesF];
    numTS = [numel(timesCC); numel(timesLC); numel(timesF)];
    
    % Evaluate cost of all terminal states.
    costTS = exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit, ...
        speedWeight,latDevWeight,timeWeight);
    
    % Generate trajectories.
    egoFrenetState = global2frenet(refPath,egoState);
    [frenetTraj,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);
    
    % Eliminate trajectories that violate constraints.
    isValid = exampleHelperEvaluateTrajectory(...
        globalTraj, maxAcceleration, maxCurvature, minVelocity);
    
    % Update the collision checker with the predicted trajectories
    % of all actors in the scene.
    for i = 1:numel(actorPoses)
        actorPoses(i).States = futureTrajectory(i).Trajectory(:,1:3);
    end
    updateObstaclePose(capList, actorID, actorPoses);
    
    % Determine evaluation order.
    [cost, idx] = sort(costTS);
    optimalTrajectory = [];
    
    trajectoryEvaluation = nan(numel(isValid),1);
    
    % Check each trajectory for collisions starting with least cost.
    for i = 1:numel(idx)
        if isValid(idx(i))
            % Update capsule list with the ego object's candidate trajectory.
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            updateEgoPose(capList, egoID, egoPoses);
            
            % Check for collisions.
            isColliding = checkCollision(capList);
            
            if all(~isColliding)
                % If no collisions are found, this is the optimal
                % trajectory.
                trajectoryEvaluation(idx(i)) = 1;
                optimalTrajectory = globalTraj(idx(i)).Trajectory;
                break;
            else
                trajectoryEvaluation(idx(i)) = 0;
            end
        end
    end
    
    % Display the sampled trajectories.
    lineHandles = exampleHelperVisualizeScene(lineHandles, globalTraj, isValid, trajectoryEvaluation);
    
    if isempty(optimalTrajectory)
        % All trajectories either violated a constraint or resulted in collision.
        %
        %   If planning failed immediately, revisit the simulator, planner,
        %   and behavior properties.
        %
        %   If the planner failed midway through a simulation, additional
        %   behaviors can be introduced to handle more complicated planning conditions.
        error('No valid trajectory has been found.');
    else
        % Visualize the scene between replanning.
        for i = (2+(0:(stepPerUpdate-1)))
            % Approximate realtime visualization.
            dt = toc;
            if scenario.SampleTime-dt > 0
                pause(scenario.SampleTime-dt);
            end
            
            egoState = optimalTrajectory(i,:);
            scenarioViewer.Actors(1).Position(1:2) = egoState(1:2);
            scenarioViewer.Actors(1).Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
            scenarioViewer.Actors(1).Yaw = egoState(3)*180/pi;
            scenarioViewer.Actors(1).AngularVelocity(3) = egoState(4)*egoState(5);
            
            % Update capsule visualization.
            hold on;
            show(capList,'TimeStep',i:capList.MaxNumSteps,'FastUpdate',1);
            hold off;
            
            % Update driving scenario.
            advance(scenarioViewer);
            tic;
        end
    end
end