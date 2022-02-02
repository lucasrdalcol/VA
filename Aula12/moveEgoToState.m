function moveEgoToState(egoVehicle, egoState)
    egoVehicle.Position(1:2) = egoState(1:2);
    egoVehicle.Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
    egoVehicle.Yaw = egoState(3)*180/pi;
    egoVehicle.AngularVelocity(3) = 180/pi*egoState(4)*egoState(5);
end