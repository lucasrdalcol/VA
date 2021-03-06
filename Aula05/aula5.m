%%
clear
close all
clc

%% Case of study
openExample('driving/VisualPerceptionUsingMonoCameraExample')

%% Exercise 1

clear, close all
clc

% Open and read video file
vFile = 'caltech_cordova1.avi';
v = VideoReader(vFile);

% Show a single frame of the Current time = 0
v.CurrentTime = 0;
vidFrame = readFrame(v);
img = imshow(vidFrame);
tt=title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', v.CurrentTime, v.FrameRate*v.CurrentTime));
% pause % press a key to continue

% While cycle to reproduce the full video
while v.hasFrame
    vidFrame = readFrame(v); % Read the next frame
    img.CData = vidFrame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', v.CurrentTime, v.FrameRate*v.CurrentTime); % Update the string
    tt.String = str;
    pause(1/v.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 2

clear, close all
clc

% Intrinsec parameters of the camera
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]

% Create instance of cameraIntrinsics using parameters above
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% define the camera orientation with respect to the vehicle's chassis for
% the Extrinsic parameters
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees

% Create instace of monoCamera. It has all the intrinsic and extrinsic
% parameters
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Open and read video file
videoName = 'caltech_cordova1.avi'; %or
% videoName = 'caltech_washington1.avi';
videoReader = VideoReader(videoName);

timeStamp = 0.06667;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame

frame = readFrame(videoReader); % read frame at timeStamp seconds
figure(1)
imshow(frame) % display frame
title('Current frame')

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

% Configure and create Bird's-Eye-View Image
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);
figure(2)
subplot(1,2,1)
imshow(birdsEyeImage)
title("Bird's-Eye-View")

% Convert to grayscale
birdsEyeImage = im2gray(birdsEyeImage);

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

% Detect lane features
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
    'ROI', vehicleROI, 'Sensitivity', laneSensitivity);
subplot(1,2,2)
imshow(birdsEyeViewBW)
title("Lane Detection")
hold on

% Obtain lane candidate points in vehicle coordinates
[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);

maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

% Obtain points of the lanes
[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

% Get points of each lane
PP1 = boundaryPoints{1};
PP2 = boundaryPoints{2};

% Convert back to images coordinate system and plot
PP1_image = vehicleToImage(birdsEyeConfig, PP1);
PP2_image = vehicleToImage(birdsEyeConfig, PP2);
plot(PP1_image(:, 1), PP1_image(:, 2), '.b')
plot(PP2_image(:, 1), PP2_image(:, 2), '.r')

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% Find short boundaries
if( numel(boundaries) > 0 )
    isOfMinLength = false(1, numel(boundaries));
    for i = 1 : numel(boundaries)
        if(diff(boundaries(i).XExtent) > minXLength)
            isOfMinLength(i) = true;
        end
    end
else
    isOfMinLength = false;
end

% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Reject short and weak boundaries
idx = 0;
strongBoundaries = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
for i = 1 : size(isOfMinLength,2)
    if( isOfMinLength(i) == 1 )
        if( boundaries(i).Strength > 0.4*maxStrength )
            idx = idx + 1;
            strongBoundaries(idx) = boundaries(i);
        end
    end
end

% Classify lane marker type when boundaryPoints are not empty
if isempty(boundaryPoints)
    strongBoundaries = repmat(strongBoundaries,1,2);
    strongBoundaries(1) = parabolicLaneBoundary(zeros(1,3));
    strongBoundaries(2) = parabolicLaneBoundary(zeros(1,3));
else
    strongBoundaries = classifyLaneTypes(strongBoundaries, boundaryPoints);
end

distancesToBoundaries = coder.nullcopy(ones(size(strongBoundaries,2),1));

% Find ego lanes
xOffset    = 0;   %  0 meters from the sensor
for i = 1 : size(strongBoundaries, 2)
    distancesToBoundaries(i) = strongBoundaries(i).computeBoundaryModel(xOffset);
end

% Find candidate ego boundaries
distancesToLeftBoundary = distancesToBoundaries>0;
if (numel(distancesToBoundaries(distancesToLeftBoundary)))
    minLeftDistance = min(distancesToBoundaries(distancesToLeftBoundary));
else
    minLeftDistance = 0;
end

distancesToRightBoundary = (distancesToBoundaries <= 0);
if( numel(distancesToBoundaries(distancesToRightBoundary)))
    minRightDistance = max(distancesToBoundaries(distancesToRightBoundary));
else
    minRightDistance = 0;
end

% Find left ego boundary
if (minLeftDistance ~= 0)
    leftEgoBoundaryIndex  = distancesToBoundaries == minLeftDistance;
    leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(leftEgoBoundaryIndex, 1)
        if( leftEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            leftEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    leftEgoBoundary = parabolicLaneBoundary.empty();
end

% Find right ego boundary
if (minRightDistance ~= 0)
    rightEgoBoundaryIndex = distancesToBoundaries == minRightDistance;
    rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
    idx = 0;
    for i = 1 : size(rightEgoBoundaryIndex, 1)
        if( rightEgoBoundaryIndex(i) == 1)
            idx = idx + 1;
            rightEgoBoundary(idx) = strongBoundaries(i);
        end
    end
else
    rightEgoBoundary = parabolicLaneBoundary.empty();
end

% Show the detected lane markers in the bird's-eye-view image and in the regular view. 
xVehiclePoints = bottomOffset:distAheadOfSensor;
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
birdsEyeWithEgoLane = insertLaneBoundary(birdsEyeWithEgoLane, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');

frameWithEgoLane = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
frameWithEgoLane = insertLaneBoundary(frameWithEgoLane, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

figure(3)
subplot('Position', [0, 0, 0.5, 1.0]) % [left, bottom, width, height] in normalized units
imshow(birdsEyeWithEgoLane)
title("Bird's-Eye-View with Ego Lane")
subplot('Position', [0.5, 0, 0.5, 1.0])
imshow(frameWithEgoLane)
title("Current frame with Ego Lane")

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector = vehicleDetectorACF();

% Width of a common vehicle is between 1.5 to 2.5 meters
vehicleWidth = [1.5, 2.5];

monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);

[bboxes, scores] = detect(monoDetector, frame);

locations = computeVehicleLocations(bboxes, sensor);

% Overlay the detections on the video frame
imgOut = insertVehicleDetections(frame, locations, bboxes);
figure(4);
imshow(imgOut);

%% Exercise 3 - my own script

clear, close all
clc

% Intrinsec parameters of the camera
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]

% Create instance of cameraIntrinsics using parameters above
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% define the camera orientation with respect to the vehicle's chassis for
% the Extrinsic parameters
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees

% Create instace of monoCamera. It has all the intrinsic and extrinsic
% parameters
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Open and read video file
videoName = 'caltech_cordova1.avi';
videoReader = VideoReader(videoName);

timeStamp = 0;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame

frame = readFrame(videoReader); % read frame at timeStamp seconds
figure(1)
subplot(1,2,1)
img_frame = imshow(frame); % display frame
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', videoReader.CurrentTime, videoReader.FrameRate*videoReader.CurrentTime));
% pause % press a key to continue

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

% Configure and create Bird's-Eye-View Image
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);
figure(1)
subplot(1,2,2)
img_birds = imshow(birdsEyeImage);
title("Bird's-Eye-View")

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

% Detect lane features
laneSensitivity = 0.25;

maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector = vehicleDetectorACF();

% Width of a common vehicle is between 1.5 to 2.5 meters
vehicleWidth = [1.5, 2.5];

monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);

% While cycle to reproduce the full video
while videoReader.hasFrame
    frame = readFrame(videoReader); % Read the next frame
    
    birdsEyeImage = transformImage(birdsEyeConfig, frame);

    % Convert to grayscale
    birdsEyeImageGray = im2gray(birdsEyeImage);
   
    % Detect lane features
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImageGray, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
        'ROI', vehicleROI, 'Sensitivity', laneSensitivity);
    
    % Obtain lane candidate points in vehicle coordinates
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
    
    % Obtain points of the lanes
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);
    
    % Find short boundaries
    if( numel(boundaries) > 0 )
        isOfMinLength = false(1, numel(boundaries));
        for i = 1 : numel(boundaries)
            if(diff(boundaries(i).XExtent) > minXLength)
                isOfMinLength(i) = true;
            end
        end
    else
        isOfMinLength = false;
    end
    
    % Reject short and weak boundaries
    idx = 0;
    strongBoundaries = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
    for i = 1 : size(isOfMinLength,2)
        if( isOfMinLength(i) == 1 )
            if( boundaries(i).Strength > 0.4*maxStrength )
                idx = idx + 1;
                strongBoundaries(idx) = boundaries(i);
            end
        end
    end
    
    % Classify lane marker type when boundaryPoints are not empty
    if isempty(boundaryPoints)
        strongBoundaries = repmat(strongBoundaries,1,2);
        strongBoundaries(1) = parabolicLaneBoundary(zeros(1,3));
        strongBoundaries(2) = parabolicLaneBoundary(zeros(1,3));
    else
        strongBoundaries = classifyLaneTypes(strongBoundaries, boundaryPoints);
    end
    
    distancesToBoundaries = coder.nullcopy(ones(size(strongBoundaries,2),1));
    
    % Find ego lanes
    xOffset    = 0;   %  0 meters from the sensor
    for i = 1 : size(strongBoundaries, 2)
        distancesToBoundaries(i) = strongBoundaries(i).computeBoundaryModel(xOffset);
    end
    
    % Find candidate ego boundaries
    distancesToLeftBoundary = distancesToBoundaries>0;
    if (numel(distancesToBoundaries(distancesToLeftBoundary)))
        minLeftDistance = min(distancesToBoundaries(distancesToLeftBoundary));
    else
        minLeftDistance = 0;
    end
    
    distancesToRightBoundary = (distancesToBoundaries <= 0);
    if( numel(distancesToBoundaries(distancesToRightBoundary)))
        minRightDistance = max(distancesToBoundaries(distancesToRightBoundary));
    else
        minRightDistance = 0;
    end
    
    % Find left ego boundary
    if (minLeftDistance ~= 0)
        leftEgoBoundaryIndex  = distancesToBoundaries == minLeftDistance;
        leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
        idx = 0;
        for i = 1 : size(leftEgoBoundaryIndex, 1)
            if( leftEgoBoundaryIndex(i) == 1)
                idx = idx + 1;
                leftEgoBoundary(idx) = strongBoundaries(i);
            end
        end
    else
        leftEgoBoundary = parabolicLaneBoundary.empty();
    end
    
    % Find right ego boundary
    if (minRightDistance ~= 0)
        rightEgoBoundaryIndex = distancesToBoundaries == minRightDistance;
        rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
        idx = 0;
        for i = 1 : size(rightEgoBoundaryIndex, 1)
            if( rightEgoBoundaryIndex(i) == 1)
                idx = idx + 1;
                rightEgoBoundary(idx) = strongBoundaries(i);
            end
        end
    else
        rightEgoBoundary = parabolicLaneBoundary.empty();
    end
    
    % Show the detected lane markers in the bird's-eye-view image and in the regular view. 
    xVehiclePoints = bottomOffset:distAheadOfSensor;
    birdsEyeImage = insertLaneBoundary(birdsEyeImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
    birdsEyeImage = insertLaneBoundary(birdsEyeImage, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
    
    frame = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frame = insertLaneBoundary(frame, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
    
    [bboxes, scores] = detect(monoDetector, frame);
    
    locations = computeVehicleLocations(bboxes, sensor);
    
    % Overlay the detections on the video frame
    frame = insertVehicleDetections(frame, locations, bboxes);

    % Update the figures handles
    img_frame.CData = frame; 
    str = sprintf('Current Time = %.3f sec, frame = %.0f', videoReader.CurrentTime, videoReader.FrameRate*videoReader.CurrentTime); % Update the string
    tt.String = str;
    
    
    img_birds.CData = birdsEyeImage;

    pause(1/videoReader.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 3 - MathWorks script with some adaptations 

clear, close all
clc

% Intrinsec parameters of the camera
focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 640];           % [nrows, mcols]

% Create instance of cameraIntrinsics using parameters above
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% define the camera orientation with respect to the vehicle's chassis for
% the Extrinsic parameters
height = 2.1798;    % mounting height in meters from the ground
pitch  = 14;        % pitch of the camera in degrees

% Create instace of monoCamera. It has all the intrinsic and extrinsic
% parameters
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Open and read video file
videoName = 'caltech_cordova1.avi'; %or
% videoName = 'caltech_washington1.avi';
videoReader = VideoReader(videoName);

timeStamp = 0;                   % time from the beginning of the video
videoReader.CurrentTime = timeStamp;   % point to the chosen frame

frame = readFrame(videoReader); % read frame at timeStamp seconds

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 30; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 6;  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

% Configure and create Bird's-Eye-View Image
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
birdsEyeImage = transformImage(birdsEyeConfig, frame);

% Lane marker segmentation ROI in world units
vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters

maxLanes      = 2; % look for maximum of two lane markers
boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width

% Establish criteria for rejecting boundaries based on their length
maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; % establish a threshold

% To compute the maximum strength, assume all image pixels within the ROI
% are lane candidate points
birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

% Convert the image points to vehicle points
vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

% Find the maximum number of unique x-axis locations possible for any lane
% boundary
maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));

% Set the maximum length of a lane boundary to the ROI length
maxLaneLength = diff(vehicleROI(1:2));

% Compute the maximum possible lane strength for this image size/ROI size
% specification
maxStrength   = maxPointsInOneLane/maxLaneLength;

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector = vehicleDetectorACF();

% Width of a common vehicle is between 1.5 to 2.5 meters
vehicleWidth = [1.5, 2.5];

monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);

isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
   
    % Grab a frame of video
    frame = readFrame(videoReader);
    
    % Compute birdsEyeView image
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = im2gray(birdsEyeImage);
    
    % Detect lane features
    laneSensitivity = 0.25;
    
    % Detect lane boundary features
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, ...
        approxLaneMarkerWidthVehicle, 'ROI', vehicleROI, ...
        'Sensitivity', laneSensitivity);

    % Obtain lane candidate points in vehicle coordinates
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);

    % Find lane boundary candidates
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);
    
    % Reject boundaries based on their length and strength    
    % Find short boundaries
    if( numel(boundaries) > 0 )
        isOfMinLength = false(1, numel(boundaries));
        for i = 1 : numel(boundaries)
            if(diff(boundaries(i).XExtent) > minXLength)
                isOfMinLength(i) = true;
            end
        end
    else
        isOfMinLength = false;
    end

    % Reject short and weak boundaries
    idx = 0;
    strongBoundaries = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
    for i = 1 : size(isOfMinLength,2)
        if( isOfMinLength(i) == 1 )
            if( boundaries(i).Strength > 0.2*maxStrength )
                idx = idx + 1;
                strongBoundaries(idx) = boundaries(i);
            end
        end
    end

    boundaries    = boundaries(isOfMinLength);
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);

    % Classify lane marker type when boundaryPoints are not empty
    if isempty(boundaryPoints)
        strongBoundaries = repmat(strongBoundaries,1,2);
        strongBoundaries(1) = parabolicLaneBoundary(zeros(1,3));
        strongBoundaries(2) = parabolicLaneBoundary(zeros(1,3));
    else
        strongBoundaries = classifyLaneTypes(strongBoundaries, boundaryPoints);
    end     
        
    % Find ego lanes
    xOffset    = 0;   %  0 meters from the sensor
    distancesToBoundaries = coder.nullcopy(ones(size(strongBoundaries,2),1));

    for i = 1 : size(strongBoundaries, 2)
        distancesToBoundaries(i) = strongBoundaries(i).computeBoundaryModel(xOffset);
    end
    % Find candidate ego boundaries
    distancesToLeftBoundary = distancesToBoundaries>0;
    if (numel(distancesToBoundaries(distancesToLeftBoundary)))
        minLeftDistance = min(distancesToBoundaries(distancesToLeftBoundary));
    else
        minLeftDistance = 0;
    end

    distancesToRightBoundary = (distancesToBoundaries <= 0);
    if( numel(distancesToBoundaries(distancesToRightBoundary)))
        minRightDistance = max(distancesToBoundaries(distancesToRightBoundary));
    else
        minRightDistance = 0;
    end

    % Find left ego boundary
    if (minLeftDistance ~= 0)
        leftEgoBoundaryIndex  = distancesToBoundaries == minLeftDistance;
        leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
        idx = 0;
        for i = 1 : size(leftEgoBoundaryIndex, 1)
            if( leftEgoBoundaryIndex(i) == 1)
                idx = idx + 1;
                leftEgoBoundary(idx) = strongBoundaries(i);
            end
        end
    else
        leftEgoBoundary = parabolicLaneBoundary.empty();
    end
    % Find right ego boundary
    if (minRightDistance ~= 0)
        rightEgoBoundaryIndex = distancesToBoundaries == minRightDistance;
        rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
        idx = 0;
        for i = 1 : size(rightEgoBoundaryIndex, 1)
            if( rightEgoBoundaryIndex(i) == 1)
                idx = idx + 1;
                rightEgoBoundary(idx) = strongBoundaries(i);
            end
        end
    else
        rightEgoBoundary = parabolicLaneBoundary.empty();
    end

    
    % Detect vehicles
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations(bboxes, sensor);
    
    % Visualize sensor outputs and intermediate results. Pack the core
    % sensor outputs into a struct.
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    
    % Pack additional visualization data, including intermediate results
    intOut.birdsEyeImage   = birdsEyeImage;    
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    
    timeStamp = 7.5333; % take snapshot for publishing at timeStamp seconds
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
end

%% Exercise 4

clear, close all
clc