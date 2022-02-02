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
scenario = aula11_scenario();

% Reference trajectory from road points
refPath = referencePathFrenet(waypoints);

% Use reference path to create the Frenet Generator
connector = trajectoryGeneratorFrenet(refPath);

scenario.SampleTime = connector.TimeResolution; %Igual o sample time do cenário ao do gerador de trajetórias
replanRate = 1; %Definir o intervalo de tempo entre planeamentos
stepPerUpdate = (1/replanRate)/scenario.SampleTime; %Definir quantos passos ocorrem entre cada replaneamento

viewer = aula11_scenario(); %Criar o Segundo cenário para representação
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
scenario = aula11_scenario_ex2();

% Reference trajectory from road points
refPath = referencePathFrenet(waypoints);

% Use reference path to create the Frenet Generator
connector = trajectoryGeneratorFrenet(refPath);

scenario.SampleTime = connector.TimeResolution; %Igual o sample time do cenário ao do gerador de trajetórias
replanRate = 1; %Definir o intervalo de tempo entre planeamentos
stepPerUpdate = (1/replanRate)/scenario.SampleTime; %Definir quantos passos ocorrem entre cada replaneamento

viewer = aula11_scenario_ex2(); %Criar o Segundo cenário para representação
viewer.SampleTime = scenario.SampleTime;

%Definir um plot do ponto de vista do veículo
chasePlot(viewer.Actors(1),'ViewLocation',-[carLen*3,0],'ViewHeight',10,'ViewPitch',20);

egoState = frenet2global(refPath, [0 0 0 laneWidth*3 0 0]); %[S dS ddS L dL ddL] -> [x y theta kappa speed accel]

terminalState = [nan speedLimit 0 laneWidth/2 0 0]; %[S dS ddS L dL ddL], o nan indica
% que a posição não é definida, use o speedLimite = 15 e o laneWidth = 3.6, a posição L ao
% longo da trajetória vai definir para que faixa o veículo se vai deslocar

time = 3; %Tempo total da trajetória a gerar
maxTimeHorizon = max(time);

capList = dynamicCapsuleList; %Define um objeto que guarda todas as geometrias a utilizar
capList.MaxNumSteps = 20;

egoID = 1;
[egoID, egoGeom] = egoGeometry(capList, egoID); %Criar a geometria do ego veículo

egoGeom.Geometry.Length = carLen; %carLen = 4.7
egoGeom.Geometry.Radius = carWidth/2; %carWidth = 1.8

updateEgoGeometry(capList, egoID, egoGeom); %Atualizar a geometria do ego veículo
actorID = 2;
actorGeom = egoGeom;
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


%% Exercise 3

clear
close all
clc

% Default car properties
carLen   = 4.7; % in meters
carWidth = 1.8; % in meters
rearAxleRatio = .25;

% Define road dimensions
laneWidth   = carWidth*2; % in meters

scenario = drivingScenarioTrafficExample; %Novo cenário

actorID = (1:5)'; %Neste cenário existem 5 atores
actorGeom = repelem(egoGeom, 5, 1);
updateObstacleGeometry(capList, actorID, actorGeom);

timeHorizons = 1:5; %Criar trajetórias em 1s e 5s de horizonte temporal
maxTimeHorizon = max(timeHorizons);

