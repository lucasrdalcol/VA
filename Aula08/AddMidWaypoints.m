function newW=AddMidWaypoints(W)
% Function to add points in the middle of every pair of points.
% W - original matrix of [x y z] points in lines
% newW - matrix with more new N-1 lines
% ...

newW = zeros(2*size(W, 1) - 1, size(W, 2));
newW(1:2:end, :) = W;

for n = 2:2:size(newW, 1)
    newW(n, :) = (newW(n-1, :) + newW(n+1, :)) / 2;
end