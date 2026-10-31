package org.firstinspires.ftc.teamcode.common.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {

    public static double CLOSE_RPM = 1680;
    public static double MID_RPM = 1900;
    public static double FAR_RPM = 2100;

    public static double hoodMin = 0.15, hoodMax = 0.57;
    public static double hoodMinDeg = 70, hoodMaxDeg = 84;

    public static double pollenPos = 0.36, nectarPos = 0.5; // 0.32 locked pollen pos

    public static double kP = 0.002, kI = 0, kD = 0, kF = 0.00024;

    public static double TICKS_PER_REV = 28;

    public static double rpmTolerance = 50;

    public static double flywheelRadiusMeters = 0.036;
    public static double goalHeightInches = 59.55;
    public static double shooterHeightInches = 9.5;

}
