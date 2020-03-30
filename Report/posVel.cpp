//Update position (ISR)
if (direction_detected == 5) position--;
else if (direction_detected == -5) position++;
else position += (direction_detected);

//Calculate velocity (motorCtrlFn)
double timeElapsed = velocityTimer.read();
velocityTimer.reset();
double velocity = (position - positionOld) * (1 / timeElapsed);
