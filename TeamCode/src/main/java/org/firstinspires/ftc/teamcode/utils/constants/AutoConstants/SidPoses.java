package org.firstinspires.ftc.teamcode.utils.constants.AutoConstants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.DashPose;

@Config
public class SidPoses {

    public static DashPose start = new DashPose(40, 16, 180);
    public static DashPose measuredStart = new DashPose(41.25, 18.15, 180);

    public static DashPose point1 = new DashPose(9, 4.5, 180);
    public static DashPose point1Control1 = new DashPose(40, 20);
    public static DashPose point1Control2 = new DashPose(15.5, 2.16);
    public static double point1TurnFrom = 180, point1TurnMid = 205, point1TurnTo = 180;

    public static DashPose point2 = new DashPose(40, 114, 90);
    public static DashPose point2Control1 = new DashPose(24, 6.75);
    public static DashPose point2Control2 = new DashPose(40, 108);
    public static double point2TurnFrom = 180, point2TurnTo = 90;

    public static DashPose point3 = new DashPose(46, 128, 90);

    public static DashPose point4 = new DashPose(13, 106.5, 180);
    public static DashPose point4Control1 = new DashPose(42, 106);
    public static double point4TurnFrom = 90, point4TurnTo = 225;

    public static DashPose point5 = new DashPose(62, 84, -90);
    public static DashPose point5Control1 = new DashPose(62, 106);

    public static DashPose point6 = new DashPose(62, 9, -90);

    public static DashPose point9 = new DashPose(14, 42, 180);
    public static DashPose point9Control1 = new DashPose(38, 28);
    public static DashPose point9Control2 = new DashPose(24, 42);
    public static double point9TurnFrom = 270, point9TurnMid = 150, point9TurnTo = 180;

    public static DashPose point10 = new DashPose(30, 14, 90);
    public static DashPose point10Control1 = new DashPose(30, 42);

    public static DashPose point11 = new DashPose(15, 92, 104.9951);
    public static DashPose point11Control1 = new DashPose(30, 36);

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
