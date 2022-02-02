function poseState = stateToPose(state)
    theta0=atan2(state(4,:),state(2,:));
    poseState = [state([1 3],:); theta0 + cumsum(state(5,:),2)*0.01]';
end