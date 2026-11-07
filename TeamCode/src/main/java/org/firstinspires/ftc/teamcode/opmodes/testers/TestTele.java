package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.robot.calculations.CalculationResult;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeTransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Config
@TeleOp(name = "TestTele")
public class TestTele extends LinearOpMode {

    public static double rpm = 0;
    public static double hood = 0.6;
    public static double turretPower = 0;
    public static Transfer.ElementMode element = Transfer.ElementMode.DETECT;
    public static double x = 0, y = 0, headingDeg = 0;

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap);
        RobotStates.shooterState = Shooter.ShooterState.MANUAL;
        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeTransferCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeTransferCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenHeld(new ShootCommand(robot));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> robot.follower.setPose(new Pose(x, y, Math.toRadians(headingDeg))));

        waitForStart();

        while (opModeIsActive()) {
            if (RobotStates.shooterState == Shooter.ShooterState.MANUAL) {
                robot.shooter.setTargetRPM(rpm);
                robot.shooter.setHoodPos(hood);
            }

            robot.turret.setManualPower(turretPower);
            robot.transfer.setElementMode(element);

            robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            CalculationResult shot = robot.shooter.lastResult;
            MyTelem.addData("pose", Robot.pose);
            MyTelem.addData("distance in", ShooterCalculator.distance(Robot.pose, Robot.getGoalPose()));
            MyTelem.addData("turret target deg", Math.toDegrees(robot.turret.getTargetAngle()));
            MyTelem.addData("turret angle deg", Math.toDegrees(robot.turret.getAngle()));
            if (RobotStates.shooterState == Shooter.ShooterState.AUTO) {
                MyTelem.addData("model hood deg", Math.toDegrees(shot.hoodAngleRad));
                MyTelem.addData("shot valid", shot.valid);
            }
            MyTelem.addData("target rpm", robot.shooter.getTargetRPM());
            MyTelem.addData("rpm", robot.shooter.getRPM());
            MyTelem.addData("at rpm", robot.shooter.atRPM());
            MyTelem.addData("left beam", robot.transfer.leftTriggered);
            MyTelem.addData("right beam", robot.transfer.rightTriggered);
            MyTelem.addData("nectar mode", robot.transfer.inNectarMode());
            robot.update();
        }

        robot.stop();
    }
}
