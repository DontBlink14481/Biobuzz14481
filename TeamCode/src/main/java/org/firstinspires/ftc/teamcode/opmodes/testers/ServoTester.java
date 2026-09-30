package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Tester")
public class ServoTester extends LinearOpMode {
    public static String name = "motor1";
    public static double position = 0.0;

    public void runOpMode(){
        Servo servo = hardwareMap.get(Servo.class, name);

        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(position);
        }
    }
}