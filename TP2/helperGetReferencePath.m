function refPathOut = helperGetReferencePath()
persistent refPath
if isempty(refPath)
% waypoints = [-43.7 52.9; 13.7 40.1; 57.1 14.5; 42.3 -63; 115.9 -120.8; 214.2 -152.7];
waypoints = [-43.7 52.9; 13.7 40.1; 57.1 14.5; 42.3 -63; 115.9 -120.8; 151 -131.2];
% waypoints = [-102.7 20.8;
%     -77.4 18.6;
%     -61.4 13.5;
%     -49.7 0.8;
%     -45.8 -21.5;
%     -27.6 -27.3;
%     -21.3 -46.1;
%     -20.8 -59.9];
% 
% waypoints = [-107.4 0.3;
%     -73.3 0;
%     -29.4 0;
%     12.1 -0.6;
%     40.9 -0.9;
%     84.5 0.3;
%     119.3 0.6];

    refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end