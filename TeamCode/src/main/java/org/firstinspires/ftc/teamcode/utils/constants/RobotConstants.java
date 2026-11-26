package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class RobotConstants {

    public static double robotWidth = 13.5;
    public static double robotLength = 13.5;

    public static double voltage = 12;
    public static double nominalVoltage = 13;

    public static double pollenDiameter = 2.8;
    public static double nectarDiameter = 3.6;

    public static Pose resetPose = new Pose(6.75, 6.75, Math.toRadians(0));

    public static Pose rightGoal = new Pose(58, 55);
    public static Pose rightAim = new Pose(58, 60.2);
    public static Pose leftAim = new Pose(58, 81.3);
    public static Pose leftGoal = new Pose(58, 86.5);

    public static double goalWidth = 20; // inches
}
