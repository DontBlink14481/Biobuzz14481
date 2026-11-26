package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

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

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
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
                Controller primaryTranslationalForward = Controller.proportional(0.26897137819982264);
                Controller secondaryTranslationalForward = Controller.proportional(0.09937773719724322);
                Controller primaryTranslationalLateral = Controller.proportional(0.5028664156557487);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18579570374670304);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01176204927578084));
                c.brake.set(Controller.proportionalFeedforward(0.009997741884413713));

                c.headingFeedback.set(Controller.proportional(3.6514565491290742));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.036919124227290774, 0.0039431184354572265));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0711283829012939, 0.018853547333767977));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016151267000682824, 0.002107544032108498));

                c.maxAchievableForwardVelocity.set(90.67810727023827);
                c.maxAchievableStrafeVelocity.set(65.71860950792389);
                c.naturalForwardDeceleration.set(39.956925302294145);
                c.naturalStrafeDeceleration.set(82.29665873926334);
            }
    );
}