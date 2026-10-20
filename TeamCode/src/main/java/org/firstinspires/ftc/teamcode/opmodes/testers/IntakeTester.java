package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeTester extends LinearOpMode {
    public void runOpMode(){
//        Robot r = new Robot(hardwareMap, false)

        waitForStart();
        while(opModeIsActive()){
            motor.setPower(power);
        }
    }
}
