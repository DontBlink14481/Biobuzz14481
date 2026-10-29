package org.firstinspires.ftc.teamcode.common.shooting;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;

public class ShooterCalculator {

    public enum Element {
        POLLEN, NECTAR
    }

    double INCHES_TO_METERS = 0.0254;
    double g = 9.8;

    public ShooterResult calculate(Pose robotPose, Pose goalPose, double rpm, Element element) {
        if (robotPose == null
                || goalPose == null
                || element == null
                || rpm <= 0) {
            return ShooterResult.invalid();
        }

        double fieldAngle = Math.atan2(
                goalPose.y() - robotPose.y(),
                goalPose.x() - robotPose.x()
        );

        double turretAngle = AngleUnit.normalizeRadians(
                fieldAngle - robotPose.heading()
        );

        //TODO: figure out how to do hood angle
        double hoodAngle = 0;

        return new ShooterResult(hoodAngle, turretAngle, true);
    }


}
