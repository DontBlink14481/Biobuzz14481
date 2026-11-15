package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeTransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp(name = "SoloTele")
public class SoloTele extends LinearOpMode {

    public static double parkTime = 5;

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        TimerEx timer = new TimerEx(120);
        Robot robot = new Robot(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenHeld(new ShootCommand(robot));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(Robot::toggleGoal);

        AllianceSelection allianceSelection = new AllianceSelection(this);

        while (opModeInInit()) {
            allianceSelection.update();

            MyTelem.addData("Alliance", Robot.alliance);
            MyTelem.update();
        }

        waitForStart();
        timer.start();

        while (opModeIsActive()) {
            DrivePowers drivePowers = new DrivePowers(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX());

            if (timer.isMoreThan(parkTime)) ManualDrive.driveOrHold(robot.follower, drivePowers);
            else robot.follower.manual(drivePowers);

            MyTelem.addData("Alliance", Robot.alliance);
            MyTelem.addData("Goal", Robot.goal);
            MyTelem.addData("target rpm", robot.shooter.getTargetRPM());
            MyTelem.addData("rpm", robot.shooter.getRPM());
            MyTelem.addData("at rpm", robot.shooter.atRPM());
            MyTelem.addData("nectar mode", robot.transfer.inNectarMode());
            MyTelem.update();

            robot.update();
        }

        robot.stop();
    }
}
