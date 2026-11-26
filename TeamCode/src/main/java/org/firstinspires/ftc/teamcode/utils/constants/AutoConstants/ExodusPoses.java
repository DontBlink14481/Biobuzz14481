package org.firstinspires.ftc.teamcode.utils.constants.AutoConstants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.DashPose;

@Config
public class ExodusPoses {
    public static DashPose start = new DashPose(56, 8, 90);
    public static DashPose point1 = new DashPose(8.75, 6.75, 180);

    public static DashPose point2 = new DashPose(42, 132, 90);
    public static DashPose point2Control1 = new DashPose(24, 6.75);
    public static DashPose point2Control2 = new DashPose(42, 120);
    public static double point2TurnFrom = 180, point2TurnTo = 90;

    public static DashPose point3 = new DashPose(42, 122, 90);

    public static DashPose point4 = new DashPose(28, 134, 180);
    public static DashPose point4Control1 = new DashPose(42, 134);

    public static DashPose point5 = new DashPose(28, 14, 90);
    public static double point5TurnFrom = 180;

    public static DashPose point6 = new DashPose(9, 42, 180);
    public static DashPose point6Control1 = new DashPose(28, 42);

    public static DashPose point7 = new DashPose(30, 14, 90);
    public static DashPose point7Control1 = new DashPose(30, 42);

    public static DashPose point8 = new DashPose(15, 92, 104.9951);
    public static DashPose point8Control1 = new DashPose(30, 36);
}
