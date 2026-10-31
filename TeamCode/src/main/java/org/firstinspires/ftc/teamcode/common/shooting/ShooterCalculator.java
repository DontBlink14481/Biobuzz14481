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

    public static double pollenCoeff = 0.88; // TODO: tune the coeffs properly these are guesses
    public static double nectarCoeff = 0.82;

    public static double minEntryDeg = 30.0;
    public static double maxEntryDeg = 89.0;

    public static ShooterResult calculate(Pose robotPose, Pose goalPose, double rpm, Element element) {
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

    public static double getLaunchVelocity(double rpm, Element element) {
        double factor = element == Element.POLLEN ? pollenCoeff : nectarCoeff;
        return rpm * 2 * Math.PI * ShooterConstants.flywheelRadiusMeters * factor / 60.0;
    }

    public static ShooterResult evaluateShot(double turretAngle, double hoodAngle, double dist, double speed, double rpm) {
        if (!inAngleRange(hoodAngle, ShooterConstants.hoodMinDeg, ShooterConstants.hoodMaxDeg)) return ShooterResult.invalid();

        double vx = speed * Math.cos(hoodAngle);
        double flightTime = dist / vx;
        double finalVy = speed * Math.sin(hoodAngle) - g * flightTime;

        double entryAngle = Math.atan2(-finalVy, vx);

        if (finalVy >= 0 || !inAngleRange(entryAngle, minEntryDeg, maxEntryDeg)) return ShooterResult.invalid();

        return new ShooterResult(turretAngle, hoodAngle, rpm, flightTime, true);
    }

    public static boolean inAngleRange(double rad, double minDeg, double maxDeg) {
        return rad >= Math.toRadians(minDeg) - EPS && rad <= Math.toRadians(maxDeg) + EPS;
    }

    public static double selectRPM(double distanceMeters, double previousRPM) {
        double closeThreshold =
                previousRPM == ShooterConstants.CLOSE_RPM ? 0.85 : 0.75;
        double farThreshold =
                previousRPM == ShooterConstants.FAR_RPM ? 1.45 : 1.55;

        if (distanceMeters < closeThreshold) return ShooterConstants.CLOSE_RPM;
        if (distanceMeters > farThreshold) return ShooterConstants.FAR_RPM;
        return ShooterConstants.MID_RPM;
    }
}