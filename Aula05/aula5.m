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
videoName = 'caltech_cordova1.avi';
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

