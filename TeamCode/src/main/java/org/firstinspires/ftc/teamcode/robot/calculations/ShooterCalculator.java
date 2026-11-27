package org.firstinspires.ftc.teamcode.robot.calculations;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
public class ShooterCalculator {

    private static final double GRAVITY = 386.09;
    private static final double DT = 0.002;

    public static double speedPerRPM = 0.0759;
    public static double lift = -0.000307;
    public static double drag = 0.001077;
    public static double launchHeight = 10.9;

    public static double angle60 = 69.35, angle66 = 71.79, angle71 = 73.55;
    private static final double[] POLLEN_DISTANCE = {30, 40, 50, 60, 70};
    private static final double[] POLLEN_HOOD = {0.66, 0.63, 0.60, 0.60, 0.60};


    private static final double[] NECTAR_DISTANCE = {30, 40, 50, 60};
    private static final double[] NECTAR_HOOD = {0.75, 0.71, 0.71, 0.60};
    private static final double[] NECTAR_RPM = {3000, 3200, 3240, 3200};

    public static double distance(Pose turretPose, Pose goalPose) {
        return Math.hypot(goalPose.x() - turretPose.x(), goalPose.y() - turretPose.y());
    }

    public static CalculationResult calculate(Pose turretPose, Pose goalPose) {
        return calculate(turretPose, goalPose, false);
    }

    public static CalculationResult calculate(Pose turretPose, Pose goalPose, boolean nectar) {
        double dx = goalPose.x() - turretPose.x();
        double dy = goalPose.y() - turretPose.y();
        double turretAngleRad = Math.atan2(dy, dx);
        double d = Math.hypot(dx, dy) + ShooterConstants.distanceOffset;

        if (nectar) {
            double hood = lookup(NECTAR_DISTANCE, NECTAR_HOOD, d);
            double rpm = lookup(NECTAR_DISTANCE, NECTAR_RPM, d);
            return new CalculationResult(hoodToAngle(hood), turretAngleRad, rpm, true);
        }

        double h = ShooterConstants.goalHeightInches - launchHeight;
        double hood = lookup(POLLEN_DISTANCE, POLLEN_HOOD, d);
        double theta = Math.toRadians(launchAngle(hood));

        // (monotomic function so we can binary search) the ball's height at the goal goes up with launch speed, so bisect on rpm
        double low = 0, high = ShooterConstants.maxRPM;
        if (heightAt(high * speedPerRPM, theta, d) < h) return CalculationResult.invalid(turretAngleRad);
        for (int i = 0; i < 16; i++) {
            double mid = (low + high) / 2;
            if (heightAt(mid * speedPerRPM, theta, d) < h) low = mid;
            else high = mid;
        }

        return new CalculationResult(hoodToAngle(hood), turretAngleRad, high, true);
    }

    // Shooter turns this angle back into the hood position
    private static double hoodToAngle(double hood) {
        double min = Math.toRadians(ShooterConstants.hoodMinDeg);
        double max = Math.toRadians(ShooterConstants.hoodMaxDeg);
        return Range.scale(hood, ShooterConstants.hoodMin, ShooterConstants.hoodMax, min, max);
    }

    private static double launchAngle(double hood) {
        if (hood <= 0.66) return Range.scale(hood, 0.60, 0.66, angle60, angle66);
        return Range.scale(hood, 0.66, 0.71, angle66, angle71);
    }

    // lienar regression for distance
    private static double lookup(double[] distances, double[] values, double d) {
        if (d <= distances[0]) return values[0];
        for (int i = 1; i < distances.length; i++) {
            if (d <= distances[i]) return Range.scale(d, distances[i - 1], distances[i], values[i - 1], values[i]);
        }
        return values[values.length - 1];
    }

    // a_x = -lift*v0*v_y - drag*|v|*v_x
    // a_y = -g + lift*v0*v_x - drag*|v|*v_y
    // height above the launch point when the ball has gone d inches forward
    private static double heightAt(double v0, double theta, double d) {
        double x = 0, y = 0;
        double vx = v0 * Math.cos(theta), vy = v0 * Math.sin(theta);
        while (x < d) {
            double speed = Math.hypot(vx, vy);
            double ax = -lift * v0 * vy - drag * speed * vx;
            double ay = -GRAVITY + lift * v0 * vx - drag * speed * vy;
            x += vx * DT;
            y += vy * DT;
            vx += ax * DT;
            vy += ay * DT;
            if (vx <= 0 || y < -launchHeight) return Double.NEGATIVE_INFINITY;
        }
        return y;
    }
}
