double posError = targetPosition - position;
double posErrorOld = targetPosition - positionOld;

return (Kpr*posError) + (Kdr*(posError - posErrorOld)/(timePassed));