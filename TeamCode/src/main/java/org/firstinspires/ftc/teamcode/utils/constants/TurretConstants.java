package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    public static double forwardPos = 0.5;
    // Turret/servo rotation = (80 / 20) * (26 / 103) = 96 / 103.
    // Assumes 360 degrees of servo travel across positions 0 through 1.
    public static double posPerDeg = (20.0 / 80.0) * (103.0 / 26.0) / 318.338; //0.22 = 90 and 0.5 = 0 done math to find that number

    public static double turretRadiansPerServoRadian() {
        return 1.0 / (360.0 * posPerDeg);
    }

    // zero is FORWARD rajveer don't fuck it up
    public static double encoderZeroVoltage = 0.9;
    public static double encoder2ZeroVoltage = 0;

    // CONTINUOUS mode PID gains (tune on the dashboard)
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.2;
    public static double targetAngleTesting = 0;

}
