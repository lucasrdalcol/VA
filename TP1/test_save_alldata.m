%% Assignment 1 - Autonomous Vehicles
% Name: Lucas Rodrigues Dal'Col
% Número Mecanográfico: 91352

clear
close all
clc

% Save data from scenario
[allData, scenario, sensors] = TP1_DSD_91352();

for n=1:numel(allData)
    allData(n).ActorPoses(2:end)=[];
end

save('allData_91352.mat', 'allData')