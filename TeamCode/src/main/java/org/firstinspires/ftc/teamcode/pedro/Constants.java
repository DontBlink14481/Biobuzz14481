package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(new OctoQuadLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(HardwareConstants.lf);
        c.frontRightName.set(HardwareConstants.rf);
        c.backLeftName.set(HardwareConstants.lb);
        c.backRightName.set(HardwareConstants.rb);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set(HardwareConstants.octoquad);
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-4.862204724409449);
        c.yPodOffset.set(-1.0236220472440944);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0129063972196226);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2143360817861447);
                Controller secondaryTranslationalForward = Controller.proportional(0.07919145505439643);
                Controller primaryTranslationalLateral = Controller.proportional(0.3644155500749445);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13464180838985843);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01276343182798837));
                c.brake.set(Controller.proportionalFeedforward(0.010848917053790114));

                c.headingFeedback.set(Controller.proportional(3.3243990297066035));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03138444496216194, 0.0032322128583479617));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0597600249393717, 0.03740420849711168));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001312741760817153, 0.0014999741133534155));

                c.maxAchievableForwardVelocity.set(82.30021910635318);
                c.maxAchievableStrafeVelocity.set(65.83484868219332);
                c.naturalForwardDeceleration.set(51.905106061212585);
                c.naturalStrafeDeceleration.set(82.56272169449684);
            }
    );
}