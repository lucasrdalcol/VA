function currentGlobalStates = updateCapsuleList(capList, predictedTracks, state2Pose)
% This is a helper function and may be removed or modified in a future
% release.

% This function takes the dynamicCapsuleList, capList and updates it with
% predictedTracks.
numTracks = size(predictedTracks, 1);
numSteps = size(predictedTracks, 2);
tgtIDs = vertcat(predictedTracks(:,1).TrackID);
tgtPoses = repmat(struct('States',zeros(numSteps,3)),numTracks,1);

currentTracks = predictedTracks(:,1);
currentTrackStates = horzcat(currentTracks.State);
currentGlobalStates = cvf2global(currentTrackStates)';

for i = 1:numTracks
    thisTrack = predictedTracks(i,:);
    thisTrackStates = horzcat(thisTrack.State);
    poseState = state2Pose(thisTrackStates);
    tgtPoses(i).States = poseState;
end

existingIDs = capList.ObstacleIDs;

% Targets removed
deletedIDs = setdiff(existingIDs, tgtIDs);
removeObstacle(capList, deletedIDs);

% Targets added
newIDs = setdiff(tgtIDs, existingIDs);
sampleGeom = getObstacleGeometry();
newGeoms = repmat(sampleGeom,numel(newIDs),1);
updateObstacleGeometry(capList,newIDs,newGeoms);

% Update pose list
updateObstaclePose(capList, tgtIDs, tgtPoses);

end

function obstacleGeom = getObstacleGeometry()
    carWidth = 1.8;
    carLen = 4.7;
    obstacleGeom.Geometry = struct('Length',2*carLen,...
        'Radius',0.8*carWidth/2,...
        'FixedTransform',eye(3));

end