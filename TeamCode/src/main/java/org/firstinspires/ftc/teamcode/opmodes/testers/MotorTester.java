package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Motor Tester")
public class MotorTester extends LinearOpMode {
    public static String name = "motor";
    public static double power = 0.0;

    public void runOpMode(){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);

        waitForStart();
        while(opModeIsActive()){
            motor.setPower(power);
        }
    }
}