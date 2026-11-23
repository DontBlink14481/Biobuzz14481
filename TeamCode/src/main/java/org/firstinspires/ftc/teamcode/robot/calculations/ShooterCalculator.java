package org.firstinspires.ftc.teamcode.robot.calculations;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

public class ShooterCalculator {

    private static final double GRAVITY = 386.09;

    public static double distance(Pose robotPose, Pose goalPose) {
        return Math.hypot(goalPose.x() - robotPose.x(), goalPose.y() - robotPose.y());
    }

    public static CalculationResult calculate(Pose robotPose, Pose goalPose) {
        double dx = goalPose.x() - robotPose.x();
        double dy = goalPose.y() - robotPose.y();
        double turretAngle = MathUtils.normalizeRadians(Math.atan2(dy, dx) - robotPose.heading(), false);

        double d = Math.hypot(dx, dy) + ShooterConstants.distanceOffset;
        double h = ShooterConstants.goalHeightInches - ShooterConstants.shooterHeightInches;

        double min = Math.toRadians(ShooterConstants.hoodMinDeg);
        double max = Math.toRadians(ShooterConstants.hoodMaxDeg);

        //tan(theta) = 2h/d + tan(arrival)
        double flattest = Math.atan(2 * h / d + Math.tan(Math.toRadians(ShooterConstants.arrivalDeg)));
        double theta = Range.clip(flattest, min, max);
        double hood = Range.scale(theta, min, max, ShooterConstants.hoodMin, ShooterConstants.hoodMax);

        // d*tan(theta) <= 2*h means the ball is still rising at the goal
        double arrival = Math.toDegrees(Math.atan(Math.tan(theta) - 2 * h / d));
        if (d * Math.tan(theta) <= h || arrival < ShooterConstants.minArrivalDeg) return CalculationResult.invalid(turretAngle);

        // v = d/cos(theta) * sqrt(g / (2*(d*tan(theta) - h)))
        // v = rpm * speedPerRPM * (1 + hoodScalar*(hood - 0.77))
        double v = d / Math.cos(theta) * Math.sqrt(GRAVITY / (2 * (d * Math.tan(theta) - h)));
        double rpm = v / (ShooterConstants.speedPerRPM * (1 + ShooterConstants.hoodScalar * (hood - 0.77)));

        return new CalculationResult(theta, turretAngle, rpm, true);
    }
}
