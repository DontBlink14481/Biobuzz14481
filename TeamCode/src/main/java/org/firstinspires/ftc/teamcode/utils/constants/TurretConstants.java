package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static double forwardPos = 0.5;
    // Turret/servo rotation = (80 / 20) * (28 / 103) = 112 / 103.
    // Assumes 360 degrees of servo travel across positions 0 through 1.
    public static double posPerDeg = (20.0 / 80.0) * (103.0 / 28.0) / 360.0;

    // zero is FORWARD rajveer don't fuck it up
    public static double encoderZeroVoltage = 0.9;
    public static double encoder2ZeroVoltage = 0;

}
