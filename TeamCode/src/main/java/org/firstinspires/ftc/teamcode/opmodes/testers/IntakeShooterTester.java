package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Intake Shooter Tester")
public class IntakeShooterTester extends LinearOpMode {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo hood;
    Servo shift;
    CRServo dropdown;

    public static boolean pollenMode = true, manualPower = false;
    public static double targetRPM = 0;
    public static double shooterPower = 0.0;
    public static double hoodPos = 0.6;

    //    DcMotorEx intake;
    DcMotorEx transfer;
    Servo pinch;

    DigitalChannel breakbeam1;
    DigitalChannel breakbeam2;

    public static boolean manualPollen = false;
    public static double transferPower = 0.0;

    public static double nectarDelayMs = 200;
    double start = nectarDelayMs;

    public void runOpMode() {
        MyTelem.init(telemetry);

        // shooter
        shooter1 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter2);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, HardwareConstants.hood);
        shift = hardwareMap.get(Servo.class, HardwareConstants.shift);

        dropdown = hardwareMap.get(CRServo.class, HardwareConstants.dropdown);

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);


//        intake = hardwareMap.get(DcMotorEx.class, HardwareConstants.intake);
        transfer = hardwareMap.get(DcMotorEx.class, HardwareConstants.transfer);

//        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        pinch = hardwareMap.get(Servo.class, HardwareConstants.pinch);

        breakbeam1 = hardwareMap.get(DigitalChannel.class, HardwareConstants.breakbeam1);
        breakbeam2 = hardwareMap.get(DigitalChannel.class, HardwareConstants.breakbeam2);

        waitForStart();

        while(opModeIsActive()) {
            if (!manualPower) {
                shooterPower = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
            }


            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            hoodPos = Math.min(ShooterConstants.hoodMax, Math.max(ShooterConstants.hoodMin, hoodPos));

            hood.setPosition(hoodPos);

            boolean leftTriggered = breakbeam1.getState();
            boolean rightTriggered = breakbeam2.getState();

            boolean triggered = leftTriggered || rightTriggered;

            if (triggered) {
                start = System.nanoTime();
            }

            double elapsed = (System.nanoTime() - start) / 1e6;

            if (!manualPollen) pollenMode = elapsed >= nectarDelayMs;


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
//        MyTelem.addData("current draw 1", shooter1.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("current draw 2", shooter2.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("total current draw", shooter1.getCurrent(CurrentUnit.MILLIAMPS) + shooter2.getCurrent(CurrentUnit.MILLIAMPS));

//            intake.setPower(transferPower);
            transfer.setPower(transferPower);
            dropdown.setPower(transferPower);

            if (pollenMode) {
                pinch.setPosition(TransferConstants.pollenPos);
            } else {
                pinch.setPosition(TransferConstants.nectarPos);
            }



            MyTelem.addData("transfer power", transfer.getPower());
            MyTelem.addData("pinch pos", pinch.getPosition());
            MyTelem.addData("left", leftTriggered);
            MyTelem.addData("right", rightTriggered);
            MyTelem.addData("manual pollen", manualPollen);
            MyTelem.addData("pollen mode", pollenMode);

            MyTelem.update();
        }
    }

    public double getRPM() {
        return (shooter1.getVelocity() * 60) / 28;
    }

}
