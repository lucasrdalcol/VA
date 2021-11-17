function boundaries = classifyLaneTypes(boundaries, boundaryPoints)

for bInd = 1 : size(boundaries,2)

    vehiclePoints = boundaryPoints{bInd};
    % Sort by x
    vehiclePoints = sortrows(vehiclePoints, 1);

    xVehicle = vehiclePoints(:,1);
    xVehicleUnique = unique(xVehicle);

    % Dashed vs solid
    xdiff  = diff(xVehicleUnique);
    % Set a threshold to remove gaps in solid line but not the spaces from
    % dashed lines.
    xdiffThreshold = mean(xdiff) + 3*std(xdiff);
    largeGaps = xdiff(xdiff > xdiffThreshold);

    % Safe default
    boundary = boundaries(bInd);           % changed according to set/get methods
    boundary.BoundaryType= LaneBoundaryType.Solid;

    if largeGaps>1
        % Ideally, these gaps should be consistent, but you cannot rely
        % on that unless you know that the ROI extent includes at least 3 dashes.
        boundary.BoundaryType= LaneBoundaryType.Dashed;
    end
    boundaries(bInd) = boundary;
end
end