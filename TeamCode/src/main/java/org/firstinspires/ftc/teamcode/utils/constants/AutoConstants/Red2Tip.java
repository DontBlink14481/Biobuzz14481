package org.firstinspires.ftc.teamcode.utils.constants.AutoConstants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class Red2Tip {
    public static final Pose START_POSE      = new Pose(33.7485, 9.4752, Math.toRadians(180));
    public static final Pose POINT_1         = new Pose(33.9936, 9.3432, Math.toRadians(180));
    public static final Pose POINT_2         = new Pose(9.1115, 9.3514, Math.toRadians(180));
    public static final Pose POINT_3         = new Pose(58.5925, 109.3435, Math.toRadians(270));
    public static final Pose POINT_4         = new Pose(3.791, 46.3452, Math.toRadians(180));
    public static final Pose POINT_5         = new Pose(58.4987, 108.0599, Math.toRadians(90));
    public static final Pose POINT_6         = new Pose(47.2136, 130.5175, Math.toRadians(90));
    public static final Pose POINT_7         = new Pose(48.692, 110.543, Math.toRadians(90));
    public static final Pose PATH3_CONTROL   = new Pose(38.0971, 138.917);
    public static final Pose PATH4_CONTROL_1 = new Pose(4.1473, 108.8969);
    public static final Pose PATH4_CONTROL_2 = new Pose(53.6806, 37.1791);
    public static final Pose PATH5_CONTROL_1 = new Pose(51.5955, 37.9119);
    public static final Pose PATH5_CONTROL_2 = new Pose(10.3292, 122.3111);
    public static final Pose PATH6_CONTROL   = new Pose(45.9013, 112.7065);

    public static Pose mirror(Pose p, String color) {
        return color.equals("BLUE") ? new Pose(141.5 - p.x(), 141.5 - p.y(), p.heading() + Math.PI) : p;
    }
}