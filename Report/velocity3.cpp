void UpdateMotorPower(float power)
{
    power = abs(power);
    if(power > 1.0) power = 1.0;

    MotorPWM.write(power)
}