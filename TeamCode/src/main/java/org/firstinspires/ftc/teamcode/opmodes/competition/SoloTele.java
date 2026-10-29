package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "SoloTele")
public class SoloTele extends OpMode {

    Robot robot;

    ElapsedTime timer;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        DrivePowers drivePowers = new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (timer.seconds() < 145)
            ManualDrive.driveOrHold(
                    robot.follower,
                    drivePowers
            );
        else
            robot.follower.manual(drivePowers);

        robot.periodic();
    }
}
