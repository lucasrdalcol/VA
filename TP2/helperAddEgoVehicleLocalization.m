function detectionsOut = helperAddEgoVehicleLocalization(detectionsIn, egoPose)

    defaultParams = struct('Frame','Rectangular',...
        'OriginPosition',zeros(3,1),...
        'OriginVelocity',zeros(3,1),...
        'Orientation',eye(3),...
        'HasAzimuth',false,...
        'HasElevation',false,...
        'HasRange',false,...
        'HasVelocity',false);
    
    fNames = fieldnames(defaultParams);
    
    detectionsOut = cell(numel(detectionsIn),1);
    
    for i = 1:numel(detectionsIn)
        thisDet = detectionsIn{i};
        if iscell(thisDet.MeasurementParameters)
            measParams = thisDet.MeasurementParameters{1};
        else
            measParams = thisDet.MeasurementParameters(1);
        end
    
        newParams = struct;
        for k = 1:numel(fNames)
            if isfield(measParams,fNames{k})
                newParams.(fNames{k}) = measParams.(fNames{k});
            else
                newParams.(fNames{k}) = defaultParams.(fNames{k});
            end
        end
    
        % Add parameters for ego vehicle
        thisDet.MeasurementParameters = [newParams;newParams];
        thisDet.MeasurementParameters(2).Frame = 'Rectangular';
        thisDet.MeasurementParameters(2).OriginPosition = egoPose.Position(:);
        thisDet.MeasurementParameters(2).OriginVelocity = egoPose.Velocity(:);
        thisDet.MeasurementParameters(2).Orientation = rotmat(quaternion([egoPose.Yaw egoPose.Pitch egoPose.Roll],'eulerd','ZYX','frame'),'frame')';
        
        
        % No information from object class and attributes
        thisDet.ObjectClassID = 0;
        thisDet.ObjectAttributes = struct;
        detectionsOut{i} = thisDet;
    end
end