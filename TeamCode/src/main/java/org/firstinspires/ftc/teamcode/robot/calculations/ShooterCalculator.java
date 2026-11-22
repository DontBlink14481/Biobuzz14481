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
        double v = ShooterConstants.baseRPM * ShooterConstants.speedPerRPM;

        // h = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta)). Swapping 1/cos^2 for 1 + tan^2
        // turns that into a quadratic in tan(theta), and the quadratic formula gives the two angles that will work!
        double discriminant = v * v * v * v - GRAVITY * (GRAVITY * d * d + 2 * h * v * v);
        double lob = Math.atan((v * v + Math.sqrt(Math.max(discriminant, 0))) / (GRAVITY * d));

        double min = Math.toRadians(ShooterConstants.hoodMinDeg);
        double max = Math.toRadians(ShooterConstants.hoodMaxDeg);
        double theta = Range.clip(lob, min, max);
        double hood = Range.scale(theta, min, max, ShooterConstants.hoodMin, ShooterConstants.hoodMax);

        // d*tan(theta) <= 2*h means the ball is still rising at the goal
        if (d * Math.tan(theta) <= 2 * h) return CalculationResult.invalid(turretAngle);

        // v = d/cos(theta) * sqrt(g / (2*(d*tan(theta) - h)))
        // v = rpm * speedPerRPM * (1 + hoodScalar*(hood - 0.77))
        v = d / Math.cos(theta) * Math.sqrt(GRAVITY / (2 * (d * Math.tan(theta) - h)));
        double rpm = v / (ShooterConstants.speedPerRPM * (1 + ShooterConstants.hoodScalar * (hood - 0.77)));

        return new CalculationResult(theta, turretAngle, rpm, true);
    }
}
