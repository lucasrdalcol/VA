function poseState = stateToPoseFrenet(state)
    cartState = cvf2global(state);
    poseState = cartState(1:3,:)';