package org.firstinspires.ftc.teamcode.utils.zones;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;

public class ShootingZone {

    public static PolygonZone rightZone;
    public static PolygonZone leftZone;

    public static double ballClearance = 1; // inches on both sides
    public static double distClearance = 10; // inches back from goal TODO: need to math this out

    public static double theta() {
        return Math.asin((RobotConstants.nectarDiameter + 2 * ballClearance) / RobotConstants.goalWidth);
    }

    public static PolygonZone right() {
        if (rightZone == null) {
            rightZone = create(RobotConstants.rightGoal, 0);
        }
        return rightZone;
    }

    public static PolygonZone left() {
        if (leftZone == null) {
            leftZone = create(RobotConstants.leftGoal, 141.5);
        }
        return leftZone;
    }

    public static PolygonZone get() {
        return Robot.goal == Robot.Goal.RIGHT ? right() : left();
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
