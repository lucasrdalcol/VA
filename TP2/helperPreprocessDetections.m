function detections = helperPreprocessDetections(detections)
    % This function pre-process the detections from radars and cameras to
    % fit the modeling assumptions used by the tracker

    % 1. It removes velocity information from camera detections. This is
    % because those are filtered estimates and the assumptions from camera
    % may not align with defined prior information for tracker.
    %
    % 2. It fixes the bias for camera sensors that arise due to camera
    % projections for cars just left or right to the ego vehicle.
    % 
    % 3. It inflates the measurement noise for range-rate reported by the
    % radars to match the range-rate resolution of the sensor
    for i = 1:numel(detections)
        if detections{i}.SensorIndex >= 1 % Camera
            % Remove velocity
            detections{i}.Measurement = detections{i}.Measurement(1:3);
            detections{i}.MeasurementNoise = blkdiag(detections{i}.MeasurementNoise(1:2,1:2),25);
            detections{i}.MeasurementParameters(1).HasVelocity = false;

            % Fix bias
            pos = detections{i}.Measurement(1:2);
            if abs(pos(1)) < 5 && abs(pos(2)) < 5
                [az, ~, r] = cart2sph(pos(1),pos(2),0);
                [pos(1),pos(2)] = sph2cart(az, 0, r + 0.7); % Increase range
                detections{i}.Measurement(1:2) = pos;
                detections{i}.MeasurementNoise(2,2) = 0.25;
            end
        else % Radars
            detections{i}.MeasurementNoise(3,3) = 0.5^2/4;
        end
    end
end