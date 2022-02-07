function state = stateTransFcn(state, w, dT)
    % Esta função define o modelo de transição de estado, como é que o próximo estado é
    %obtido com base no estado atual
    if isscalar(w)
    w = repmat(w,[4 size(state,2)]);
    end
    tau = 3;
    state(1,:) = state(1,:) + state(2,:)*dT + w(1,:)*dT^2/2;
    state(2,:) = state(2,:) + w(1,:)*dT;
    state(3,:) = state(3,:) + state(4,:)*tau*(1 - exp(-dT/tau)) + w(2,:)*dT^2/2;
    state(4,:) = state(4,:)*exp(-dT/tau) + w(2,:)*dT;
    end