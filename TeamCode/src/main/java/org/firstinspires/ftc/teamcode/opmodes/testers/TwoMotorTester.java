package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Two Motor Tester")
public class TwoMotorTester extends LinearOpMode {
    public static String name = "motor1", name2 = "motor2";
    public static MotorDirections direction = MotorDirections.SAME;
    public static double power = 0.0;

    public void runOpMode(){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, name2);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        switch(direction) {
            case SAME:
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case OPPOSITE:
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
        }

        waitForStart();
        while(opModeIsActive()){
            motor.setPower(power);
            motor2.setPower(power);
        }
    }

    enum MotorDirections{
        OPPOSITE, SAME
    }
}