package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TransferConstants;

@Config
@TeleOp(name = "Intake Shooter Tester")
public class IntakeShooterTester extends LinearOpMode {
    DcMotorEx shooter;
    Servo hood;
    Servo shift;

    public static boolean pollenMode = true, manualPower = false;
    public static double targetRPM = 0;
    public static double shooterPower = 0.0;
    public static double hoodPos = 0.6;

    DcMotorEx intake;
    DcMotorEx transfer;
    Servo pinch;

    DigitalChannel breakbeam1;
    DigitalChannel breakbeam2;

    public static boolean manualPollen = false;
    public static double transferPower = 0.0;

    public static double nectarDelayMs = 25;
    public static double nectarHoodOffset = 0.1;
    double start = nectarDelayMs;

    public void runOpMode() {
        MyTelem.init(telemetry);

        // shooter
        shooter = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter1);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, HardwareConstants.hood);
        hood.setDirection(Servo.Direction.REVERSE);
        shift = hardwareMap.get(Servo.class, HardwareConstants.shift);

        intake = hardwareMap.get(DcMotorEx.class, HardwareConstants.intake);
        transfer = hardwareMap.get(DcMotorEx.class, HardwareConstants.transfer);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        pinch = hardwareMap.get(Servo.class, HardwareConstants.pinch);

        breakbeam1 = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak1);
        breakbeam2 = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak2);

        waitForStart();

        while(opModeIsActive()) {
            if (!manualPower) {
                shooterPower = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
            }


            shooter.setPower(shooterPower);

            double hoodNectarPos = hoodPos + nectarHoodOffset;
            double adjHoodPos = pollenMode ? hoodPos : hoodNectarPos;
            adjHoodPos = Math.min(ShooterConstants.hoodMax, Math.max(ShooterConstants.hoodMin, adjHoodPos));

            hood.setPosition(adjHoodPos);

            boolean leftTriggered = breakbeam1.getState();
            boolean rightTriggered = breakbeam2.getState();

            boolean triggered = leftTriggered /*|| rightTriggered*/;

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

            MyTelem.addData("power", shooter.getPower());
            MyTelem.addData("target rpm", targetRPM);
            MyTelem.addData("rpm", getRPM());
            MyTelem.addData("hood pos", hood.getPosition());
            MyTelem.addData("hood insert pos", hoodPos);
            MyTelem.addData("current draw 1", shooter.getCurrent(CurrentUnit.AMPS));
            intake.setPower(transferPower);
            transfer.setPower(transferPower);

            if (pollenMode) {
                pinch.setPosition(TransferConstants.pollenPos);
            } else {
                pinch.setPosition(TransferConstants.nectarPos);
            }



            MyTelem.addData("transfer power", transfer.getPower());
            MyTelem.addData("transfer amps", transfer.getCurrent(CurrentUnit.AMPS));
            MyTelem.addData("pinch pos", pinch.getPosition());
            MyTelem.addData("left", leftTriggered);
            MyTelem.addData("right", rightTriggered);
            MyTelem.addData("manual pollen", manualPollen);
            MyTelem.addData("pollen mode", pollenMode);

            MyTelem.update();
        }
    }

    public double getRPM() {
        return -(shooter.getVelocity() * 60) / 28;
    }

}