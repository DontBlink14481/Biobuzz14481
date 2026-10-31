package org.firstinspires.ftc.teamcode.common.shooting;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;

@Config
public class ShooterCalculator {

    public enum Element {
        POLLEN, NECTAR
    }

    private static final double g = 9.81;
    private static final double inchesToMeters = 0.0254;
    private static final double EPS = 1e-9;

    public static double minShotDistMeters = 0.5;

    public static double pollenCoeff = 1.0; // TODO: tune the coeffs
    public static double nectarCoeff = 1.0;

    public static double minEntryDeg = 30.0;
    public static double maxEntryDeg = 60.0;

    public ShooterResult calculate(Pose robotPose, Pose goalPose, double rpm, Element element) {
        if (rpm <= 0) return ShooterResult.invalid();

        double dx = goalPose.x() - robotPose.x();
        double dy = goalPose.y() - robotPose.y();

        double fieldAngle = Math.atan2(dy, dx);
        double turretAngle = AngleUnit.normalizeRadians(fieldAngle - robotPose.heading());

        double dist = Math.hypot(dx, dy) * inchesToMeters;
        double height = (ShooterConstants.goalHeightInches - ShooterConstants.shooterHeightInches) * inchesToMeters;

        if (dist < minShotDistMeters) return ShooterResult.invalid();

        double speed = getLaunchVelocity(rpm, element);

        double speedSquared = speed * speed;
        double numerator = g * dist * dist + 2 * height * speedSquared;
        double discriminant = speedSquared * speedSquared - g * numerator;

        if (discriminant < 0) return ShooterResult.invalid();

        double root = Math.sqrt(discriminant);

        double lowAngle = Math.atan2(numerator, dist * (speedSquared + root));
        double highAngle = Math.atan2(speedSquared + root, g * dist);

        ShooterResult lowShot = evaluateShot(turretAngle, lowAngle, dist, speed, rpm);
        ShooterResult highShot = evaluateShot(turretAngle, highAngle, dist, speed, rpm);

        return lowShot.valid ? lowShot : highShot;
    }

    public double getLaunchVelocity(double rpm, Element element) {
        double factor = element == Element.POLLEN ? pollenCoeff : nectarCoeff;
        return rpm * Math.PI * ShooterConstants.flywheelRadiusMeters * factor / 60.0;
    }

    public ShooterResult evaluateShot(double turretAngle, double hoodAngle, double dist, double speed, double rpm) {
        if (!inAngleRange(hoodAngle, ShooterConstants.hoodMinDeg, ShooterConstants.hoodMaxDeg)) return ShooterResult.invalid();

        double vx = speed * Math.cos(hoodAngle);
        double flightTime = dist / vx;
        double finalVy = speed * Math.sin(hoodAngle) - g * flightTime;

        double entryAngle = Math.atan2(-finalVy, vx);

        if (finalVy >= 0 || !inAngleRange(entryAngle, minEntryDeg, maxEntryDeg)) return ShooterResult.invalid();

        return new ShooterResult(turretAngle, hoodAngle, rpm, flightTime, true);
    }

    public boolean inAngleRange(double rad, double minDeg, double maxDeg) {
        return rad >= Math.toRadians(minDeg) - EPS && rad <= Math.toRadians(maxDeg) + EPS;
    }
}