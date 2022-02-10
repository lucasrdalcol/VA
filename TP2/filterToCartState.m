function cartState = filterToCartState(filterState)
    % Assemble as Frenet state to use frenet2global
    numStates = size(filterState,2);
    s = filterState(1,:);
    ds = filterState(2,:);
    dds = zeros(1,numStates);
    d = filterState(3,:);
    dd = filterState(4,:);
    ddbyds = zeros(1,numStates);
    ddbyds(abs(ds) > 0.5) = dd(abs(ds) > 0.5)./ds(abs(ds) > 0.5);
    dd2ds2 = zeros(1,numStates);
    frenetState = [s;ds;dds;d;ddbyds;dd2ds2];
    % Convert to global state
    refPath = helperGetReferencePath();
    try
        globalState = frenet2global(refPath,frenetState')';
    catch
        frenetState(1) = 0;
        globalState = frenet2global(refPath,frenetState')';
    end
    % Convert to cartesian state
    x = globalState(1,:);
    y = globalState(2,:);
    speed = globalState(5,:);
    theta = globalState(3,:);
    vx = speed.*cos(theta);
    vy = speed.*sin(theta);
    cartState = [x;vx;y;vy];
end