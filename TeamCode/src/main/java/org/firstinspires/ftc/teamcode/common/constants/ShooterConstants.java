package org.firstinspires.ftc.teamcode.common.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {

    public static double maxRPM = 2300;

    public static double hoodMin = 0.15, hoodMax = 0.57;
    public static double hoodMinDeg = 70, hoodMaxDeg = 84;

    public static double pollenPos = 0.36, nectarPos = 0.5; // 0.32 locked pollen pos

    public static double kP = 0.01, kI = 0.0, kD = 0.0001, kF = 0.0;

    public static double TICKS_PER_REV = 28;

    public static double rpmTolerance = 50;

    public static double goalHeightInches = 59.55;
    public static double shooterHeightInches = 9.5;

}
