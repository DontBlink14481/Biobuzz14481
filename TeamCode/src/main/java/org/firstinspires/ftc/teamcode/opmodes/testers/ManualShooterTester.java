package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp(name = "Manual Shooter Tester")
public class ManualShooterTester extends LinearOpMode {
    DcMotorEx shooter;
    Servo hood;
    Servo shift;

    public static boolean pollenMode = true, manualPower = false;
    public static double targetRPM = 0;
    public static double power = 0.0;
    public static double hoodPos = 0.6;

    public void runOpMode(){
        MyTelem.init(telemetry);

        shooter = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, HardwareConstants.hood);
        hood.setDirection(Servo.Direction.REVERSE);
        shift = hardwareMap.get(Servo.class, HardwareConstants.shift);

        waitForStart();

        while(opModeIsActive()) {
            if (!manualPower) {
                power = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
            }

            shooter.setPower(power);

            hoodPos = Math.min(ShooterConstants.hoodMax, Math.max(ShooterConstants.hoodMin, hoodPos));

            hood.setPosition(hoodPos);

            if (pollenMode) {
                shift.setPosition(ShooterConstants.pollenPos);
            } else {
                shift.setPosition(ShooterConstants.nectarPos);
            }

            MyTelem.addData("power", shooter.getPower());
            MyTelem.addData("target rpm", targetRPM);
            MyTelem.addData("rpm", getRPM());
            MyTelem.addData("hood pos", hood.getPosition());
            MyTelem.addData("hood insert pos", hoodPos);
            MyTelem.addData("current draw", shooter.getCurrent(CurrentUnit.MILLIAMPS));
            MyTelem.update();
        }
    }

    public double getRPM() {
        return -(shooter.getVelocity() * 60) / 28;
    }

}