package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {

    public static double maxRPM = 6000;
    public static double rpmTolerance = 50;
    public static double kF = 0;
    public static double spinUpTimeoutS = 1.5;

    public static double hoodMin = 0.6, hoodMax = 0.71;
    public static double hoodMinDeg = 63, hoodMaxDeg = 70;
    public static double arrivalDeg = 35, minArrivalDeg = -20;

    public static double pollenPos = 0.25, nectarPos = 0.39;

    public static double goalHeightInches = 59.55;
    public static double shooterHeightInches = 9.5;

    public static double targetRPM = 0;

    public static double baseRPM = 3250;
    public static double speedPerRPM = 0.068;
    public static double hoodScalar = 0.72;
    public static double distanceOffset = 0;

}
