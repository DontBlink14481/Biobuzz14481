package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.skeletonarmy.marrow.TimerEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp
public class Zero2 extends LinearOpMode {
    public static double parkTime = 5;
    private boolean manualHood = false;
    public static double driveExpo = 2;
    public static double turnScale = 1.5;
    public static double aimTol = 2;
    public static double hoodPos = (ShooterConstants.hoodMin + ShooterConstants.hoodMax) / 2;
    public Intake.ElementMode elementMode = Intake.ElementMode.DETECT;

    public static double curve(double input) {
        return Math.copySign(Math.pow(Math.abs(input),driveExpo), input);
    }

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        TimerEx timer = new TimerEx(120);
        Pose startPose = Robot.pose;
        Robot robot = new Robot(hardwareMap);
        robot.follower.setPose(startPose);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator=new GamepadEx(gamepad2);

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new TurretCommand(robot, Turret.TurretState.SET))
                .whenReleased(new TurretCommand(robot, Turret.TurretState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new ParallelCommandGroup(
//                        new TurretCommand(robot, Turret.TurretState.SET),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.shooter.setHoodPos(hoodPos)),
                                        new ShooterCommand(robot, Shooter.ShooterState.MANUAL)
                                ),
                                new ShooterCommand(robot, Shooter.ShooterState.AUTO),
                                () -> manualHood
                        )
                ))
                .whenHeld(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> Math.abs(MathUtils.normalizeRadians(
                                robot.turret.getTargetAngle() - robot.turret.getAngle(), false)) < Math.toRadians(aimTol)),
                        new ShootCommand(robot)
                ))
                .whenReleased(new ParallelCommandGroup(
//                        new TurretCommand(robot, Turret.TurretState.OFF),
                        new ShooterCommand(robot, Shooter.ShooterState.OFF),
                        new IntakeCommand(robot, Intake.IntakeState.OFF),
                        new InstantCommand(() -> robot.shooter.setTargetRPM(0))
                ));

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new InstantCommand(() -> manualHood=!manualHood));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new ParallelCommandGroup(
                        new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.ON),
                        new IntakeCommand(robot, Intake.IntakeState.IN)
                ))
                .whenReleased(new ParallelCommandGroup(
                        new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.OFF),
                        new IntakeCommand(robot, Intake.IntakeState.OFF)
                ));

        AllianceSelection allianceSelection = new AllianceSelection(this);
        while(opModeInInit()) {
            allianceSelection.update();
            MyTelem.addData("Alliance", Robot.alliance);
            MyTelem.update();
        }

        waitForStart();
        timer.start();
        while(opModeIsActive()) {
            DrivePowers drivePowers = new DrivePowers(
                    curve(driver.getLeftY()),
                    curve(-driver.getLeftX()),
                    Range.clip(curve(-driver.getRightX()) * turnScale, -1, 1));

            if (timer.isMoreThan(parkTime)) ManualDrive.driveOrHold(robot.follower, drivePowers);
            else {robot.follower.manual(drivePowers);}
            MyTelem.addData("Alliance", Robot.alliance);
            MyTelem.addData("pose", Robot.pose);
            MyTelem.addData("Goal", Robot.goal);
            MyTelem.addData("turret state", RobotStates.turretState);
            MyTelem.addData("turret angle", robot.turret.getAngle(AngleUnit.DEGREES));
            MyTelem.addData("target rpm", robot.shooter.getTargetRPM());
            MyTelem.addData("rpm", robot.shooter.getRPM());
            MyTelem.addData("at rpm", robot.shooter.atRPM());
            MyTelem.addData("shooter state", RobotStates.shooterState);
            MyTelem.addData("shot valid", robot.shooter.lastResult.valid);
            MyTelem.addData("hood mode", manualHood ? "MANUAL" : "AUTO");
            MyTelem.addData("manual hood pos", hoodPos);
            MyTelem.addData("element mode", elementMode);
            MyTelem.addData("nectar mode", robot.intake.inNectarMode());
            MyTelem.update();
            robot.update();
        }
        robot.stop();
    }
}