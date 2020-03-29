
float DistanceError(double timePassed)
{
    // get current and previous distacne error
    targetPosition_mutex.lock();
    maxSpeed_mutex.lock();

    double posError = targetPosition - position;
    double posErrorOld = targetPosition - positionOld;

    maxSpeed_mutex.unlock();
    targetPosition_mutex.unlock();

    return (Kpr*posError) + (Kdr*(posError - posErrorOld)/(timePassed));
}