
float DistanceError(double timePassed)
{
    // get current and previous distacne error
    targetPosition_mutex.lock();

    double posError = targetPosition - position;
    double posErrorOld = targetPosition - positionOld;

    targetPosition_mutex.unlock();

    return (Kpr*posError) + (Kdr*(posError - posErrorOld)/(timePassed));
}
