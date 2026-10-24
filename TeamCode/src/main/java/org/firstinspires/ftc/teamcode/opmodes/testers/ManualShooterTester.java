package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Manual Shooter Tester")
public class ManualShooterTester extends LinearOpMode {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo hood;
    Servo shift;
    CRServo dropdown;

    public static boolean pollenMode = true, manualPower = false;
    public static double targetRPM = 0;
    public static double power = 0.0;
    public static double hoodPos = 0.6;

    public void runOpMode(){
        MyTelem.init(telemetry);

        shooter1 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter2);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, HardwareConstants.hood);
        shift = hardwareMap.get(Servo.class, HardwareConstants.shift);

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            if (!manualPower) {
                power = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
            }


            shooter1.setPower(power);
            shooter2.setPower(power);

            hoodPos = Math.min(ShooterConstants.hoodMax, Math.max(ShooterConstants.hoodMin, hoodPos));

            hood.setPosition(hoodPos);

            if (pollenMode) {
                shift.setPosition(ShooterConstants.pollenPos);
            } else {
                shift.setPosition(ShooterConstants.nectarPos);
            }

            MyTelem.addData("power", shooter1.getPower());
            MyTelem.addData("target rpm", targetRPM);
            MyTelem.addData("rpm", getRPM());
            MyTelem.addData("hood pos", hood.getPosition());
            MyTelem.addData("hood insert pos", hoodPos);
    //        MyTelem.addData("current draw 1", motor1.getCurrent(CurrentUnit.MILLIAMPS));
    //        MyTelem.addData("current draw 2", motor2.getCurrent(CurrentUnit.MILLIAMPS));
    //        MyTelem.addData("total current draw", motor1.getCurrent(CurrentUnit.MILLIAMPS) + motor2.getCurrent(CurrentUnit.MILLIAMPS));
            MyTelem.update();
        }
    }

    public double getRPM() {
        return (shooter1.getVelocity() * 60) / 28;
    }

}
