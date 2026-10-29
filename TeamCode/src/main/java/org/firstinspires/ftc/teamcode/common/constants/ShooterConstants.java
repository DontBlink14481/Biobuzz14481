package org.firstinspires.ftc.teamcode.common.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {

    public static double hoodMin = 0.35, hoodMax = 0.8;

    public static double pollenPos = 0.36, nectarPos = 0.5; // 0.32 locked pollen pos

    public static double kP = 0, kI = 0, kD = 0, kF = 0;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(kP, kI, kD, kF);

    public static double TICKS_PER_REV = 28;

    public static double rpmTolerance = 10;

    public static double goalHeightInches = 59.55;
    //TODO: measure flywheel height or ask karthik
    public static double shooterHeightInches = 8;

}
