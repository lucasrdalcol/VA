%% 
clear
close
clc

% Aula02 - LiDAR 2D

%% Scan 2D - Exercise 1
load('lidarScans.mat')

size(lidarScans);

% Get scan number 250
s = lidarScans(250);

% Represent point cloud of the scan s using cartesian coordinates
sCart = s.Cartesian;

figure(1)
subplot(1,2,1)
plot(sCart(:, 1), sCart(:, 2), '.k')
hold on
plot(0, 0, '.g') % Plot origin, where is the scan
axis equal
grid on
axis([-5 5 -8 3])
xlabel('x')
ylabel('y')

% Represent point cloud of the scan s using polar coordinates to verify is
% the same as the previous
[x, y] = pol2cart(s.Angles, s.Ranges);
plot(x, y, 'or')

% Plot now using the method from the class lidarScan to plot automatically
subplot(1,2,2)
h = s.plot;
hold on
plot(0, 0, '.g') % Plot origin, where is the scan
% Rotate 90 degrees conter-clockwise to align with ROS coordinate frame
view(0,90)
axis equal
grid on
axis([-5 5 -8 3])

%% Exercise 2

% Animate how the robot is going through the scene

% Coordinates of the triangle that represents the robot
robot=[  0    0.5  0
        -0.2  0    0.2];

% Initialize a array to save the count of points in each scan
totSavedScans = [];

% Establish ranges to remove invalid data from the scans
minRange = 0.15;
maxRange = 8;

% Cycle all scans
for n=1:numel(lidarScans)
    % Get current scan
    scan = lidarScans(n);

    % Remove invalid data according to the ranges specified.
    scan = removeInvalidData(scan, 'RangeLimits', [minRange maxRange]);
    
    % Get cartesian coordinates of the current scan and plot it.
    scanCart = scan.Cartesian;
    totSavedScans = [totSavedScans, scan.Count]; % Update array of point count
    hold off
    figure(2)
    scan.plot; % Plot the current scan
    axis([-8 8 -8 8]);
    hold on
    fill([0; scanCart(:,1)], [0; scanCart(:,2)], 'y'); % Fill the place where the robot is
    fill(robot(1,:), robot(2,:),'r'); % Fill the triangle that represents the robot
    pause(0.01)
end

% Get the minimum number of points of all scans and find each scans are
minPoints = min(totSavedScans);
max_n_points_removed = lidarScans(1).Count - minPoints
scans_idx = find(totSavedScans==minPoints)

%% Exercise 3
% Animate how the robot is going through the scene ploting the previous
% scan and the current scan.

load lidarScans.mat;
numScans = numel(lidarScans);

% Coordinates of the triangle that represents the robot
robot=[  0    0.5  0
        -0.2  0    0.2];

figure(3)

step = 1; % step between previous scan and current scan

% Cycle the scans according to the step choosen 
for n = step+1:numScans
    % Get previous and current scan according to the step choosen
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    
    % Plot previous and current scan
    hold off
    h1 = prevScan.plot; 
    h1.MarkerEdgeColor = 'k'; 
    axis([-8 8 -8 8]);
    hold on

    h2 = currScan.plot; 
    h2.MarkerEdgeColor = 'r'; 
    axis([-8 8 -8 8]);
    
    fill(robot(1,:), robot(2,:), 'c'); % Fill the triangle that represents the robot
    legend('Reference (previous) scan', 'Current scan', 'Robot');
    title(sprintf('Scan %d and %d of %d', n-step, n, numScans))
    pause(0.001)
end

%% Exercise 3b
% Animate how the robot is going through the scene ploting the previous
% scan, the current scan and the scan according to the transformation
% between them.

load lidarScans.mat;
numScans = numel(lidarScans);

% Coordinates of the triangle that represents the robot
robot=[  0    0.5  0
        -0.2  0    0.2];

figure(4)

step = 1; % step between previous scan and current scan

% Cycle the scans according to the step choosen 
for n = step+1:numScans
    % Get previous and current scan according to the step choosen
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    
    % Get the pose from previous to current scan and estimate the
    % transformation between them to plot it
    [pose, stats] = matchScans(currScan,prevScan);
    estScan = transformScan(prevScan,pose);
    
    % Plot previous, current and estimated scans
    hold off
    h1 = prevScan.plot; 
    h1.MarkerEdgeColor = 'k'; 
    axis([-8 8 -8 8]);
    hold on

    h2 = currScan.plot; 
    h2.MarkerEdgeColor = 'r'; 
    axis([-8 8 -8 8]);
    
    fill(robot(1,:), robot(2,:), 'c');

    h3 = estScan.plot;
    h3.MarkerEdgeColor = 'b';
    axis([-8 8 -8 8]);

    str = sprintf('Scan=%d, Score= %d, Pose=[%.0f mm, %.0f mm, %.2f??]', n, round(stats.Score), pose .* [1000 1000 180/pi]); 
    title(str)
    subtitle('Pose is of current scan relative to previous (reference) scan')

    legend('Reference (previous) scan', 'Current scan', 'Robot', 'Estimated scan')
    pause(0.01)
end

%% Exercise 4
% Estimate the robot's trajectory


load lidarScans.mat;
numScans = numel(lidarScans);

% Coordinates of the triangle that represents the robot
robot=[  0    0.5  0
        -0.2  0    0.2];

initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;

step = 1; % step between previous scan and current scan

% Establish ranges to remove invalid data from the scans
min_range = 0.15;
max_range = 8;

% Cycle the scans according to the step choosen 
for n = step+1:numScans
    % Get previous and current scan according to the step choosen
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);

    % Remove invalid data according to the ranges specified.
    prevScan = removeInvalidData(prevScan, 'RangeLimits', [min_range max_range]);
    currScan = removeInvalidData(currScan, 'RangeLimits', [min_range max_range]);
    
    % Get the pose from previous to current scan and estimate the
    % transformation between them
    [pose, stats] = matchScans(currScan,prevScan);
    poseList(n,:) = pose;
end

figure(5)
hold on
grid on
fill(robot(1,:), robot(2,:), 'c');

T = eye(3);
for n = step+1:size(poseList,1)
    T = T * homoTransform2d(poseList(n,:)); %T is the accumulated total
    
    pose = [T(1,3), T(2,3), atan2(T(2,1),T(1,1))]; %pose is the current iteration real pose
    
    % Transform the points of the robot to the current absolute pose
    robotf = T * [robot; 1 1 1];
    
    fill(robotf(1,:), robotf(2,:), 'c');
    title('Estimated robot trajectory')
    axis([-2 6 -5 2]);
    pause(0.01)
end


%% Exercise 5
% Occupancy grid plot

load lidarScans.mat;
numScans = numel(lidarScans);

% Coordinates of the triangle that represents the robot
robot=[  0    0.5  0
        -0.2  0    0.2];

initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;
scoreList = zeros(numScans);

step = 1; % step between previous scan and current scan

% Establish ranges to remove invalid data from the scans
min_range = 0.15;
max_range = 8;

% Get occupancy map
map = occupancyMap(15,15,20);
map.GridLocationInWorld = [-7.5 -7.5];

% Cycle the scans according to the step choosen 
for n = step+1:numScans
    % Get previous and current scan according to the step choosen
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);

    % Remove invalid data according to the ranges specified.
    prevScan = removeInvalidData(prevScan, 'RangeLimits', [min_range max_range]);
    currScan = removeInvalidData(currScan, 'RangeLimits', [min_range max_range]);
    
    % Get the pose from previous to current scan and estimate the
    % transformation between them
    [pose, stats] = matchScans(currScan,prevScan);
    poseList(n,:) = pose;

    insertRay(map, poseList(n, :), currScan, 10);
end

figure(5)
show(map)
hold on
grid on
fill(robot(1,:), robot(2,:), 'c');


T = eye(3);
for n = step+1:size(poseList,1)
    T = T * homoTransform2d(poseList(n,:)); %T is the accumulated total
    
    pose = [T(1,3), T(2,3), atan2(T(2,1),T(1,1))]; %pose is the current iteration real pose
    
    % Transform the points of the robot to the current absolute pose
    robotf = T * [robot; 1 1 1];
    
    fill(robotf(1,:), robotf(2,:), 'c');
    title('Estimated robot trajectory')
    
    pause(0.01)
end

plot(poseList(:,1), poseList(:,2), 'bo', 'DisplayName', 'Estimated robot position');

