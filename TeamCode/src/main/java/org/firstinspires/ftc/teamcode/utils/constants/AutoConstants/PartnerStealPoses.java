package org.firstinspires.ftc.teamcode.utils.constants.AutoConstants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.DashPose;

@Config
public class PartnerStealPoses {

    public static DashPose start = new DashPose(40, 16, 180);
    public static DashPose measuredStart = new DashPose(41.25, 18.25, 180);

    public static DashPose point1 = new DashPose(0, 4.5, 180);
    public static DashPose point1Control1 = new DashPose(40, 20);
    public static DashPose point1Control2 = new DashPose(15.5, 2.16);
    public static double point1TurnFrom = 180, point1TurnMid = 205, point1TurnTo = 180;

    public static DashPose point2 = new DashPose(40, 114, 90);
    public static DashPose point2Control1 = new DashPose(24, 6.75);
    public static DashPose point2Control2 = new DashPose(40, 108);
    public static double point2TurnFrom = 180, point2TurnTo = 90;

    public static DashPose point3 = new DashPose(44, 125, 90);

    public static DashPose point4 = new DashPose(42, 118, 90);

    public static DashPose point5 = new DashPose(28, 134, 180);
    public static DashPose point5Control1 = new DashPose(42, 134);

    public static DashPose point6 = new DashPose(28, 14, 90);
    public static double point6TurnFrom = 180;

    public static DashPose point7 = new DashPose(14, 42, 180);
    public static DashPose point7Control1 = new DashPose(28, 42);

    public static DashPose point8 = new DashPose(30, 14, 90);
    public static DashPose point8Control1 = new DashPose(30, 42);

    public static DashPose point9 = new DashPose(15, 92, 104.9951);
    public static DashPose point9Control1 = new DashPose(30, 36);

    public static Pose routePose(DashPose point) {
        return routePose(point, point.headingDeg);
    }

    public static Pose routePose(DashPose point, double headingDeg) {
        return new Pose(
                point.x + (measuredStart.x - start.x),
                point.y + (measuredStart.y - start.y),
                Math.toRadians(headingDeg)
        );
    }
}
