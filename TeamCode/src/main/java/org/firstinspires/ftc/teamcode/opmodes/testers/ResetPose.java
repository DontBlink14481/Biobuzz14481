package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class ResetPose extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        robot.follower.setPose(new Pose(72, 72, 0));

        waitForStart();
    }
}