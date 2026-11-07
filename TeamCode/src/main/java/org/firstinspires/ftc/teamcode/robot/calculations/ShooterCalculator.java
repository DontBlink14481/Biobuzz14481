package org.firstinspires.ftc.teamcode.robot.calculations;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

public class ShooterCalculator {

    private static final double GRAVITY = 386.09;

    public static double distance(Pose robotPose, Pose goalPose) {
        return Math.hypot(goalPose.x() - robotPose.x(), goalPose.y() - robotPose.y());
    }

    public static CalculationResult calculate(Pose robotPose, Pose goalPose) {
        double dx = goalPose.x() - robotPose.x();
        double dy = goalPose.y() - robotPose.y();
        double turretAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robotPose.heading());

        double d = Math.hypot(dx, dy);
        double h = ShooterConstants.goalHeightInches - ShooterConstants.shooterHeightInches;
        double v = ShooterConstants.launchSpeed;

        // The ball's path gives h = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta)). Swapping 1/cos^2 for 1 + tan^2
        // turns that into a quadratic in tan(theta), and the quadratic formula gives the two angles that hit.
        double discriminant = v * v * v * v - GRAVITY * (GRAVITY * d * d + 2 * h * v * v);
        if (d <= 0 || discriminant < 0) return CalculationResult.invalid(turretAngle);

        double lob = Math.atan((v * v + Math.sqrt(discriminant)) / (GRAVITY * d));
        double flat = Math.atan((v * v - Math.sqrt(discriminant)) / (GRAVITY * d));

        double hoodAngle = inHoodRange(lob) ? lob : flat;
        if (!inHoodRange(hoodAngle)) return CalculationResult.invalid(turretAngle);

        return new CalculationResult(hoodAngle, turretAngle, true);
    }

    private static boolean inHoodRange(double angle) {
        return angle >= Math.toRadians(ShooterConstants.hoodMinDeg) && angle <= Math.toRadians(ShooterConstants.hoodMaxDeg);
    }
}
