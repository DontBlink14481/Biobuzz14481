package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Intake Tester")
public class IntakeTester extends LinearOpMode {

//    DcMotorEx intake;
    DcMotorEx transfer;
    Servo pinch;

    DigitalChannel left;
    DigitalChannel right;

    public static boolean manualPollen = false;
    public static boolean pollenMode = false;
    public static double transferPower = 0.0;

    public static double nectarDelayMs = 200;
    double start = nectarDelayMs;

    public void runOpMode() {
        MyTelem.init(telemetry);

//        intake = hardwareMap.get(DcMotorEx.class, HardwareConstants.intake);
        transfer = hardwareMap.get(DcMotorEx.class, HardwareConstants.transfer);

//        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        pinch = hardwareMap.get(Servo.class, HardwareConstants.pinch);

        left = hardwareMap.get(DigitalChannel.class, "left");
        right = hardwareMap.get(DigitalChannel.class, "right");

        waitForStart();

        while(opModeIsActive()) {
//            intake.setPower(power);
            transfer.setPower(transferPower);


            if (pollenMode) {
                pinch.setPosition(IntakeConstants.pollenPos);
            } else {
                pinch.setPosition(IntakeConstants.nectarPos);
            }

            boolean leftTriggered = left.getState();
            boolean rightTriggered = right.getState();

            boolean triggered = leftTriggered || rightTriggered;

            if (triggered) {
                start = System.nanoTime();
            }

            double elapsed = (System.nanoTime() - start) / 1e6;

            if (!manualPollen) pollenMode = elapsed >= nectarDelayMs;

            MyTelem.addData("transfer power", transfer.getPower());
            MyTelem.addData("pinch pos", pinch.getPosition());
            MyTelem.addData("left", leftTriggered);
            MyTelem.addData("right", rightTriggered);
            MyTelem.addData("manual pollen", manualPollen);
            MyTelem.addData("pollen mode", pollenMode);
            MyTelem.update();
        }
    }
}
