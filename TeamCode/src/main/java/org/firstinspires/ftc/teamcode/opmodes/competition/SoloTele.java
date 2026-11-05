package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.commands.subsystem.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp(name = "SoloTele")
public class SoloTele extends OpMode {

    Robot robot;

    GamepadEx gp1;
    GamepadEx gp2;

    ElapsedTime timer;

    public static double parkTime = 5;

    @Override
    public void init() {
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));


        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        DrivePowers drivePowers = new DrivePowers(-gp1.getLeftY(), -gp1.getLeftX(), -gp1.getRightX());

        if (timer.seconds() < (120 - parkTime))
            ManualDrive.driveOrHold(
                    robot.follower,
                    drivePowers
            );
        else robot.follower.manual(drivePowers);



        robot.periodic();
    }
}
