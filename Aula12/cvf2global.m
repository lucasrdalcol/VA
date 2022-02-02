function globalState = cvf2global(trackStates)
if isempty(trackStates)
    globalState = zeros(0,6);
    return;
end
refPath = helperGetReferencePath();
numStates = size(trackStates,2);
s = trackStates(1,:);
ds = trackStates(2,:);
dds = zeros(1,numStates);
L = trackStates(3,:);
hasNonZeroSpeed = abs(ds) > 1;
dLds = zeros(1,numStates);
dLds(hasNonZeroSpeed) = trackStates(4,hasNonZeroSpeed)./ds(hasNonZeroSpeed);
dL2ds2 = zeros(1,numStates);
frenetStates = [s;ds;dds;L;dLds;dL2ds2];
globalState = frenet2global(refPath,frenetStates')';
end