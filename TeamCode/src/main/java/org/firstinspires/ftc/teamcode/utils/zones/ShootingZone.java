package org.firstinspires.ftc.teamcode.utils.zones;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;

public class ShootingZone {

    public static PolygonZone frontZone;
    public static PolygonZone backZone;

    public static double ballClearance = 1; // inches on both sides
    public static double distClearance = 10; // inches back from goal TODO: need to math this out

    public static double theta() {
        return Math.asin((RobotConstants.nectarDiameter + 2 * ballClearance) / RobotConstants.goalWidth);
    }

    public static PolygonZone front() {
        if (frontZone == null) {
            frontZone = create(RobotConstants.frontGoal, 0);
        }
        return frontZone;
    }

    public static PolygonZone back() {
        if (backZone == null) {
            backZone = create(RobotConstants.backGoal, 141.5);
        }
        return backZone;
    }

    public static PolygonZone create(Pose goal, double farY) {
        double closeY = goal.y() + Math.copySign(distClearance, farY - goal.y());
        double slant = 1 / Math.tan(theta());
        double closeHalf = distClearance * slant;
        double farHalf = Math.abs(farY - goal.y()) * slant;

        return new PolygonZone(
                new Point(goal.x() - closeHalf, closeY),
                new Point(goal.x() + closeHalf, closeY),
                new Point(goal.x() + farHalf, farY),
                new Point(goal.x() - farHalf, farY));
    }
}
