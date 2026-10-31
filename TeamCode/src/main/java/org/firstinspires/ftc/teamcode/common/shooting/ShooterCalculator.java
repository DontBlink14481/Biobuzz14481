package org.firstinspires.ftc.teamcode.common.shooting;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;

import java.util.ArrayList;
import java.util.List;

@Config
public class ShooterCalculator {

    public enum Element {
        POLLEN, NECTAR
    }

    static double g = 9.81;
    static double inchesToMeters = 0.0254;
    static double rpmEpsilon = 1e-6;
    static int searchSteps = 128;
    static int iterations = 60;

    public static double pollenA = 0, pollenB = 0, pollenC = 0, pollenD = 0, pollenE = 1;
    public static double nectarA = 0, nectarB = 0, nectarC = 0, nectarD = 0, nectarE = 1;

    public static ShooterResult calculate(Pose robotPose, Pose goalPose, double rpm, Element element) {
        double dx = goalPose.x() - robotPose.x();
        double dy = goalPose.y() - robotPose.y();
        double turretAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robotPose.heading());
        if (rpm <= 0 || rpm > ShooterConstants.maxRPM) return ShooterResult.invalid(turretAngle);

        double distance = Math.hypot(dx, dy) * inchesToMeters;
        ShotCurve curve = makeCurve(distance, element);
        if (curve == null) return ShooterResult.invalid(turretAngle);

        double angle = curve.angleAtRPM(rpm);
        if (!Double.isFinite(angle)) return ShooterResult.invalid(turretAngle);

        double speed = curve.speed(angle);
        double flightTime = curve.distance / (speed * Math.cos(angle));
        return new ShooterResult(turretAngle, angle, rpm, flightTime, true);
    }

    public static double selectRPM(double distanceMeters, double previousRPM) {
        previousRPM = Math.min(previousRPM, ShooterConstants.maxRPM);
        double rpm = findSharedRPM(distanceMeters, previousRPM);
        if (rpm > 0) return rpm;

        ShotCurve pollen = makeCurve(distanceMeters, Element.POLLEN);
        if (pollen != null) {
            rpm = chooseRPM(pollen.minRPM, pollen.maxRPM, previousRPM);
            if (rpm > 0 && Double.isFinite(pollen.angleAtRPM(rpm))) return rpm;
        }

        // Keep spinning if even pollen has no solution. Each element is validated separately.
        return previousRPM;
    }

    static double findSharedRPM(double distanceMeters, double previousRPM) {
        ShotCurve pollen = makeCurve(distanceMeters, Element.POLLEN);
        ShotCurve nectar = makeCurve(distanceMeters, Element.NECTAR);
        if (pollen == null || nectar == null) return 0;

        double rpm = chooseRPM(Math.max(pollen.minRPM, nectar.minRPM),
                Math.min(pollen.maxRPM, nectar.maxRPM), previousRPM);
        if (rpm <= 0) return 0;

        return Double.isFinite(pollen.angleAtRPM(rpm)) && Double.isFinite(nectar.angleAtRPM(rpm)) ? rpm : 0;
    }

    static double chooseRPM(double min, double max, double previousRPM) {
        min = Math.max(0, min);
        max = Math.min(ShooterConstants.maxRPM, max);
        if (max <= 0 || min > max) return 0;

        return previousRPM > 0 && previousRPM >= min && previousRPM <= max
                ? previousRPM : (min + max) / 2;
    }

    static ShotCurve makeCurve(double distance, Element element) {
        double height = (ShooterConstants.goalHeightInches - ShooterConstants.shooterHeightInches) * inchesToMeters;
        double minAngle = Math.toRadians(ShooterConstants.hoodMinDeg);
        double maxAngle = Math.toRadians(ShooterConstants.hoodMaxDeg);
        double minPosition = ShooterConstants.hoodMin;
        double maxPosition = ShooterConstants.hoodMax;

        if (distance <= 0) return null;

        double a = element == Element.POLLEN ? pollenA : nectarA;
        double b = element == Element.POLLEN ? pollenB : nectarB;
        double c = element == Element.POLLEN ? pollenC : nectarC;
        double d = element == Element.POLLEN ? pollenD : nectarD;
        double e = element == Element.POLLEN ? pollenE : nectarE;

        double factorMin = Math.min((c * minPosition + d) * minPosition + e,
                (c * maxPosition + d) * maxPosition + e);
        if (c > 0) {
            double vertex = -d / (2 * c);
            if (vertex > minPosition && vertex < maxPosition) {
                factorMin = Math.min(factorMin, (c * vertex + d) * vertex + e);
            }
        }

        double speedLimit = (ShooterConstants.maxRPM / factorMin - b) / a * inchesToMeters;
        if (speedLimit <= 0) return null;
        double speedSquared = speedLimit * speedLimit;
        double discriminant = speedSquared * speedSquared - g * (g * distance * distance + 2 * height * speedSquared);
        if (discriminant < 0) return null;
        double root = Math.sqrt(discriminant);
        double lowAngle = Math.atan2(g * distance * distance + 2 * height * speedSquared,
                distance * (speedSquared + root));
        double highAngle = Math.atan2(speedSquared + root, g * distance);
        double lower = Math.max(minAngle, lowAngle);
        double upper = Math.min(maxAngle, highAngle);
        if (lower > upper) return null;

        ShotCurve curve = new ShotCurve(distance, height, minAngle, maxAngle,
                minPosition, maxPosition, a, b, c, d, e);
        curve.findTurns(lower, upper);
        return curve;
    }

    static class ShotCurve {

        double distance, height, minAngle, maxAngle, minPosition, maxPosition;
        double a, b, c, d, e;
        List<Double> turns = new ArrayList<>();
        double minRPM = Double.POSITIVE_INFINITY;
        double maxRPM = Double.NEGATIVE_INFINITY;

        ShotCurve(double distance, double height, double minAngle, double maxAngle, double minPosition, double maxPosition, double a, double b, double c, double d, double e) {
            this.distance = distance;
            this.height = height;
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.e = e;
        }

        double position(double angle) {
            return minPosition + (angle - minAngle) * (maxPosition - minPosition) / (maxAngle - minAngle);
        }

        double speed(double angle) {
            double cos = Math.cos(angle);
            return Math.sqrt(g * distance * distance / (2 * cos * cos * (distance * Math.tan(angle) - height)));
        }

        double rpm(double angle) {
            double q = position(angle);
            return (a * speed(angle) / inchesToMeters + b) * ((c * q + d) * q + e);
        }

        double derivative(double angle) {
            double q = position(angle);
            double cos = Math.cos(angle);
            double velocity = speed(angle) / inchesToMeters;
            double velocityDerivative = velocity * (Math.tan(angle)
                    - distance / (2 * cos * cos * (distance * Math.tan(angle) - height)));
            double positionDerivative = (maxPosition - minPosition) / (maxAngle - minAngle);
            return a * velocityDerivative * ((c * q + d) * q + e)
                    + (a * velocity + b) * (2 * c * q + d) * positionDerivative;
        }

        void addTurn(double angle) {
            turns.add(angle);
            double rpm = rpm(angle);
            minRPM = Math.min(minRPM, rpm);
            maxRPM = Math.max(maxRPM, rpm);
        }

        void findTurns(double lower, double upper) {
            // Split at extrema, so a nonmonotonic hood regression can have multiple solutions.
            addTurn(lower);
            double left = lower;
            double leftDerivative = derivative(left);
            for (int i = 1; i <= searchSteps; i++) {
                double right = i == searchSteps ? upper : lower + (upper - lower) * i / searchSteps;
                double rightDerivative = derivative(right);
                if (leftDerivative == 0) {
                    if (left > lower) addTurn(left);
                } else if (Math.signum(leftDerivative) != Math.signum(rightDerivative)) {
                    double lo = left;
                    double hi = right;
                    for (int j = 0; j < iterations; j++) {
                        double mid = (lo + hi) / 2;
                        if (Math.signum(derivative(mid)) == Math.signum(leftDerivative)) lo = mid;
                        else hi = mid;
                    }
                    addTurn((lo + hi) / 2);
                }
                left = right;
                leftDerivative = rightDerivative;
            }
            addTurn(upper);
        }

        double angleAtRPM(double target) {
            for (int i = 1; i < turns.size(); i++) {
                double lo = turns.get(i - 1);
                double hi = turns.get(i);
                double loRPM = rpm(lo);
                double hiRPM = rpm(hi);
                if (Math.abs(loRPM - target) <= rpmEpsilon) return lo;
                if (Math.abs(hiRPM - target) <= rpmEpsilon) return hi;
                if (target < Math.min(loRPM, hiRPM) || target > Math.max(loRPM, hiRPM)) continue;

                boolean increasing = hiRPM > loRPM;
                for (int j = 0; j < iterations; j++) {
                    double mid = (lo + hi) / 2;
                    if ((rpm(mid) < target) == increasing) lo = mid;
                    else hi = mid;
                }
                double angle = (lo + hi) / 2;
                if (Math.abs(rpm(angle) - target) <= rpmEpsilon) return angle;
            }
            return Double.NaN;
        }
    }
}
