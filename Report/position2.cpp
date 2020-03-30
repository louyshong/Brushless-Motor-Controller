//find distance error
DE = DistanceError(timeElapsed);

// add non-linearity to prevent motor oscillation at small speeds (kills the error if within 0.5 rots)
if(abs(targetPosition - position) < 3)
{
    DE = 0;
}

//find velocity error
VE = VelocityError(abs(velocity));
if(VE < 0) VE = 0;

//choose error to pass to motor
float power_test = std::min(abs(VE), abs(DE));
