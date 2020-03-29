//find distance error
DE = DistanceError((double)velocityTimer.read());

//find velocity error
VE = VelocityError(abs(velocity));

//choose error to pass to motor
float power_test = 0;
if (velocity >= 0)
    power_test = std::min(VE,DE);
else
    power_test = std::max(VE,DE);