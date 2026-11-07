package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeTransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@TeleOp(name = "SoloTele")
public class SoloTele extends LinearOpMode {

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new ShootCommand(robot));

        waitForStart();

        while (opModeIsActive()) {
            robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            MyTelem.addData("target rpm", robot.shooter.getTargetRPM());
            MyTelem.addData("rpm", robot.shooter.getRPM());
            MyTelem.addData("at rpm", robot.shooter.atRPM());
            MyTelem.addData("nectar mode", robot.transfer.inNectarMode());
            robot.update();
        }

        robot.stop();
    }
}
