%% 
clear
close
clc

%% Scan 2D
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
xlabel('x')
ylabel('y')

% Represent point cloud of the scan s using polar coordinates to verify is
% the same as the previous
[x, y] = pol2cart(s.Angles, s.Ranges);
plot(x, y, 'or')

subplot(1,2,2)
h = s.plot;
hold on
plot(0, 0, '.g') % Plot origin, where is the scan
view(0,90)
axis equal
grid on
axis([-5 5 -8 3])


subplot(1,2,1) %change to the original plot
axis([-5 5 -8 3])

%% Exercise 2

% Animate how the robot is going through the scene

robot=[  0    0.5  0
        -0.2  0    0.2];
totSavedScans = [];

for n=1:numel(lidarScans)
    scan = lidarScans(n);
    minRange = 0.15;
    maxRange = 8;
    scan = removeInvalidData(scan, 'RangeLimits', [minRange maxRange]);
    
    scanCart = scan.Cartesian;
    totSavedScans = [totSavedScans, size(scanCart,1)];
    hold off
    figure(2)
    scan.plot;
    axis([-8 8 -8 8]);
    hold on
    fill([0; scanCart(:,1)], [0; scanCart(:,2)], 'y');
    fill(robot(1,:), robot(2,:),'r');
    pause(0.01)
end

minPoints = min(totSavedScans);
max_n_points_removed = scan.Count - minPoints
scans_idx = find(totSavedScans==minPoints)

%% Exercise 3

load lidarScans.mat;
numScans = numel(lidarScans);

robot=[  0    0.5  0
        -0.2  0    0.2];

figure(3)
for n = 2:numScans
    step = 1;
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    
    hold off
    h1 = prevScan.plot; 
    h1.MarkerEdgeColor = 'k'; 
    axis([-8 8 -8 8]);
    hold on

    h2 = currScan.plot; 
    h2.MarkerEdgeColor = 'r'; 
    axis([-8 8 -8 8]);
    
    fill(robot(1,:), robot(2,:),'c');
    legend('Reference (previous)', 'Current', 'Robot');
    title(sprintf('Scan %d and %d of %d', n-step, n, numScans))
    pause(0.01)
end

%% Exercise 3b

load lidarScans.mat;
numScans = numel(lidarScans);

robot=[  0    0.5  0
        -0.2  0    0.2];

figure(4)
step = 1;
for n = step+1:numScans
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);

    [pose, stats] = matchScans(currScan,prevScan);
    estScan = transformScan(prevScan,pose);

    hold off
    h1 = prevScan.plot; 
    h1.MarkerEdgeColor = 'k'; 
    axis([-8 8 -8 8]);
    hold on

    h2 = currScan.plot; 
    h2.MarkerEdgeColor = 'r'; 
    axis([-8 8 -8 8]);
    
    fill(robot(1,:), robot(2,:),'c');

    h3 = estScan.plot;
    h3.MarkerEdgeColor = 'b';
%     h3.Marker = 'o';
    axis([-8 8 -8 8]);

    str=sprintf('Scan=%d, Score= %d, Pose=[%.0f mm, %.0f mm, %.2fº]', n, round(stats.Score), pose .* [1000 1000 180/pi]); 
    title(str)
    subtitle('Pose is of current scan relative to previous (reference) scan')

    legend('Reference (previous)', 'Current', 'Robot')
    pause(0.01)
end

%% Exercise 4

load lidarScans.mat;
numScans = numel(lidarScans);

robot=[  0    0.5  0
        -0.2  0    0.2];

initialPose = [0 0 0]; %initial estimate of pose
poseList = zeros(numScans,3); %empty list to calculate poses
poseList(1,:) = initialPose;
scoreList = zeros(numScans);


step = 1;
min_range = 0.15;
max_range = 8;
for n = step+1:numScans
    prevScan = lidarScans(n-step);
    currScan = lidarScans(n);
    prevScan = removeInvalidData(prevScan, 'RangeLimits', [minRange maxRange]);
    currScan = removeInvalidData(currScan, 'RangeLimits', [minRange maxRange]);

    [pose, stats] = matchScans(currScan,prevScan);
    poseList(n,:) = pose;
end

figure(5)
hold on
grid on

T = eye(3);
for n = 2:size(poseList,1)
    T = T * homoTransform2d(poseList(n,:)); %T is the accumulated total

    pose = [T(1,3), T(2,3), atan2(T(2,1),T(1,1)) ]; %pose is the current iteration real pose
    fill(pose * robot(1,:), pose * robot(2,:),'c');
end


