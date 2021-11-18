%%
clear
close all
clc

%% Exercise 1

clear, close all
clc

% Load image
img = imread('highway.png');
figure(1)
subplot(1,2,1)
imshow(img) % display image
title('Original Image')

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector_car = vehicleDetectorACF();

% Apply detector
[bboxes, scores] = detect(detector_car, img);

% Insert bounding box in the image
img = insertObjectAnnotation(img, 'rectangle', bboxes, scores);

% Display image with cars detected
subplot(1,2,2)
imshow(img)
title('Image with cars detections')

%% Exercise 2

clear, close all
clc

% Open and read video file
vFile = 'test_youtube.mp4';
video = VideoReader(vFile);

% Show a single frame of the Current time = 0
video.CurrentTime = 0;
frame = readFrame(video);
img = imshow(frame);
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime));
% pause % press a key to continue

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector_car = vehicleDetectorACF();

% While cycle to reproduce the full video
while video.hasFrame
    frame = readFrame(video); % Read the next frame

    % Apply detector
    [bboxes, scores] = detect(detector_car, frame);
    
    % Insert bounding box in the image
    frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores);

    img.CData = frame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime); % Update the string
    tt.String = str;
    pause(1/video.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 3

clear, close all
clc

% Open and read video file
vFile = 'test_youtube.mp4';
video = VideoReader(vFile);

% Show a single frame of the Current time = 0
video.CurrentTime = 0;
frame = readFrame(video);
img = imshow(frame);
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime));
% pause % press a key to continue

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector_car = vehicleDetectorACF();

% While cycle to reproduce the full video
while video.hasFrame
    frame = readFrame(video); % Read the next frame

    % Apply detector
    [bboxes, scores] = detect(detector_car, frame);
    
    % Filter the detections applying a threshold in the scores array
    FF = scores>20;
    
    % Separate between good and bad detections. Filter the bboxes and the
    % scores array
    good_bboxes_cars = bboxes;
    good_scores_cars = scores;
    good_bboxes_cars(~FF, :) = [];
    good_scores_cars(~FF, :) = [];

    bad_bboxes_cars = bboxes;
    bad_scores_cars = scores;
    bad_bboxes_cars(FF, :) = [];
    bad_scores_cars(FF, :) = [];
    
    % Insert bounding boxes in the image
    if numel(bboxes)>0
        if numel(good_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', good_bboxes_cars, good_scores_cars, 'Color', 'green');
        end
        if numel(bad_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', bad_bboxes_cars, bad_scores_cars, 'Color', 'red');
        end
    end

    img.CData = frame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime); % Update the string
    tt.String = str;
    pause(1/video.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 4

clear, close all
clc

% Open and read video file
vFile = 'test_youtube.mp4';
video = VideoReader(vFile);

% Start video with Current time = 30
video.CurrentTime = 30;
frame = readFrame(video);
img = imshow(frame);
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime));
% pause % press a key to continue

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
% detector = peopleDetectorACF('inria-100x41');
detector_person = peopleDetectorACF('caltech-50x21');

detector_car = vehicleDetectorACF();


% While cycle to reproduce the full video
while video.hasFrame
    frame = readFrame(video); % Read the next frame

    % Apply detectors
    [bboxes_cars, scores_cars] = detect(detector_car, frame);
    [bboxes_people, scores_people] = detect(detector_person, frame);
    
    % Filter the detections applying a threshold in the scores array
    FF = scores_cars > 20;
    
    % Separate between good and bad detections. Filter the bboxes and the
    % scores array
    good_bboxes_cars = bboxes_cars;
    good_scores_cars = scores_cars;
    good_bboxes_cars(~FF, :) = [];
    good_scores_cars(~FF, :) = [];

    bad_bboxes_cars = bboxes_cars;
    bad_scores_cars = scores_cars;
    bad_bboxes_cars(FF, :) = [];
    bad_scores_cars(FF, :) = [];
    
    % Insert bounding boxes of cars in the image
    if numel(bboxes_cars)>0
        if numel(good_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', good_bboxes_cars, good_scores_cars, 'Color', 'green');
        end
        if numel(bad_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', bad_bboxes_cars, bad_scores_cars, 'Color', 'red');
        end
    end

    if numel(bboxes_people) > 0
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes_people, scores_people, 'Color', 'yellow');
    end

    img.CData = frame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime); % Update the string
    tt.String = str;
    pause(1/video.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 5

clear, close all
clc

% Open and read video file
vFile = 'test_youtube.mp4';
video = VideoReader(vFile);

% Show a single frame of the Current time = 0
video.CurrentTime = 0;
frame = readFrame(video);
img = imshow(frame);
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime));
% pause % press a key to continue

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector_car = vehicleDetectorFasterRCNN();

% While cycle to reproduce the full video
while video.hasFrame
    frame = readFrame(video); % Read the next frame

    % Apply detector
    [bboxes, scores] = detect(detector_car, frame);
    
    % Filter the detections applying a threshold in the scores array
    FF = scores>0.8;
    
    % Separate between good and bad detections. Filter the bboxes and the
    % scores array
    good_bboxes_cars = bboxes;
    good_scores_cars = scores;
    good_bboxes_cars(~FF, :) = [];
    good_scores_cars(~FF, :) = [];

    bad_bboxes_cars = bboxes;
    bad_scores_cars = scores;
    bad_bboxes_cars(FF, :) = [];
    bad_scores_cars(FF, :) = [];
    
    % Insert bounding boxes in the image
    if numel(bboxes)>0
        if numel(good_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', good_bboxes_cars, good_scores_cars, 'Color', 'green');
        end
        if numel(bad_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', bad_bboxes_cars, bad_scores_cars, 'Color', 'red');
        end
    end

    img.CData = frame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime); % Update the string
    tt.String = str;
    pause(1/video.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end

%% Exercise 6

clear, close all
clc

% Open and read video file
vFile = 'test_youtube.mp4';
video = VideoReader(vFile);

% Show a single frame of the Current time = 0
video.CurrentTime = 0;
frame = readFrame(video);
img = imshow(frame);
tt = title(sprintf('Current Time = %.3f sec, Current Frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime));
% pause % press a key to continue

% Detecting and tracking vehicles is critical in front collision warning (FCW) and autonomous emergency braking (AEB) systems.
detector_car = vehicleDetectorACF();

% get centroid of the bounding box and accumulate
good_centroids_accumulate = [];
bad_centroids_accumulate = [];

% While cycle to reproduce the full video
while video.hasFrame
    frame = readFrame(video); % Read the next frame

    % Apply detector
    [bboxes, scores] = detect(detector_car, frame);
    
    % Filter the detections applying a threshold in the scores array
    FF = scores>20;
    
    % Separate between good and bad detections. Filter the bboxes and the
    % scores array
    good_bboxes_cars = bboxes;
    good_scores_cars = scores;
    good_bboxes_cars(~FF, :) = [];
    good_scores_cars(~FF, :) = [];

    bad_bboxes_cars = bboxes;
    bad_scores_cars = scores;
    bad_bboxes_cars(FF, :) = [];
    bad_scores_cars(FF, :) = [];

    % Get current centroid of the good bounding boxes
    good_centroids = good_bboxes_cars;
    good_centroids = round(good_centroids(:, 1:2) + good_centroids(:, 3:4)/2);

    % Get current centroid of the bad bounding boxes
    bad_centroids = bad_bboxes_cars;
    bad_centroids = round(bad_centroids(:, 1:2) + bad_centroids(:, 3:4)/2);
    
    % Insert bounding boxes in the image
    if numel(bboxes)>0
        if numel(good_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', good_bboxes_cars, good_scores_cars, 'Color', 'green');
            hold on
            plot(good_centroids(:, 1), good_centroids(:, 2), '.g')
        end
        if numel(bad_bboxes_cars)>0
            frame = insertObjectAnnotation(frame, 'rectangle', bad_bboxes_cars, bad_scores_cars, 'Color', 'red');
            plot(bad_centroids(:, 1), bad_centroids(:, 2), '.r')
        end
    end

    img.CData = frame; % Update the figure handle
    str = sprintf('Current Time = %.3f sec, frame = %.0f', video.CurrentTime, video.FrameRate*video.CurrentTime); % Update the string
    tt.String = str;
    pause(1/video.FrameRate) % To use the same velocity of the video, use the inverse of the frameRate
end
