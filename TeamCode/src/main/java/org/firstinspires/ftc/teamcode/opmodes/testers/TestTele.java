package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.robot.calculations.CalculationResult;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Config
@TeleOp(name = "TestTele")
public class TestTele extends LinearOpMode {

    public static double rpm = 2500;
    public static double hood = 0.4;
    public static double turretPosition = 0.5;
    public static Intake.ElementMode element = Intake.ElementMode.POLLEN;
    public static double x = 0, y = 0, headingDeg = 0;

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap);
        RobotStates.shooterState = Shooter.ShooterState.MANUAL;
        robot.shooter.setTargetRPM(rpm);
        RobotStates.turretState = Turret.TurretState.MANUAL;
        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        driver.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenHeld(new ShootCommand(robot));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> robot.follower.setPose(new Pose(x, y, Math.toRadians(headingDeg))));

        waitForStart();

        while (opModeIsActive()) {
            if (RobotStates.shooterState == Shooter.ShooterState.MANUAL) {
                robot.shooter.setTargetRPM(rpm);
                robot.shooter.setHoodPos(hood);
                robot.intake.setState(Intake.IntakeState.IN);
                robot.blocker.setState(Blocker.BlockerState.UNBLOCKED);
            }

//            robot.turret.setManualPower(turretPosition);
            robot.intake.setElementMode(element);

            robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            CalculationResult shot = robot.shooter.lastResult;
            MyTelem.addData("pose", Robot.pose);
            MyTelem.addData("pose x in", Robot.pose.x());
            MyTelem.addData("pose y in", Robot.pose.y());
            MyTelem.addData("pose heading deg", Math.toDegrees(Robot.pose.heading()));
            MyTelem.addData("turret state", RobotStates.turretState);
            MyTelem.addData("goal", Robot.goal);
//            MyTelem.addData("turret position", robot.turret.getPosition());
//            MyTelem.addData("turret servo 2 position", robot.turret.getServo2Position());
            MyTelem.addData("turret error deg", AngleUnit.normalizeDegrees(
                    Math.toDegrees(robot.turret.getTargetAngle() - robot.turret.getAngle())));
            MyTelem.addData("distance in", ShooterCalculator.distance(Robot.pose, Robot.getGoalPose()));
            MyTelem.addData("turret target deg", Math.toDegrees(robot.turret.getTargetAngle()));
            MyTelem.addData("calculated target deg", Math.toDegrees(
                    ShooterCalculator.calculate(Robot.pose, Robot.getGoalPose()).turretAngleRad));
            MyTelem.addData("turret aim updates", robot.turret.getAimUpdateCount());
            MyTelem.addData("goal dx in", Robot.getGoalPose().x() - Robot.pose.x());
            MyTelem.addData("goal dy in", Robot.getGoalPose().y() - Robot.pose.y());
            MyTelem.addData("turret angle deg", Math.toDegrees(robot.turret.getAngle()));
            MyTelem.addData("turret encoder voltage", robot.turret.getEncoderVoltage());
            MyTelem.addData("turret encoder 2 voltage", robot.turret.getEncoder2Voltage());
            MyTelem.addData("turret encoder 2 angle deg", robot.turret.getEncoder2Angle(AngleUnit.DEGREES));
            if (RobotStates.shooterState == Shooter.ShooterState.AUTO) {
                MyTelem.addData("model hood deg", Math.toDegrees(shot.hoodAngleRad));
                MyTelem.addData("shot valid", shot.valid);
            }
            MyTelem.addData("target rpm", robot.shooter.getTargetRPM());
            MyTelem.addData("rpm", robot.shooter.getRPM());
            MyTelem.addData("at rpm", robot.shooter.atRPM());
            MyTelem.addData("left beam", robot.intake.leftTriggered);
            MyTelem.addData("right beam", robot.intake.rightTriggered);
            MyTelem.addData("nectar mode", robot.intake.inNectarMode());
            robot.update();
        }

        robot.stop();
    }
}
