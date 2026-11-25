package org.firstinspires.ftc.teamcode.opmodes.teleOp;

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
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.ShooterCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TurretCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

@Config
@TeleOp(name="MainTele")
public class MainTele extends LinearOpMode {
    public static double parkTime=5;
    public static double startX = 72, startY = 72, startHeadingDeg = 0;
    private boolean manualHood = false;
    public static double driveExpo=2;
    public static double hoodPos=(ShooterConstants.hoodMin+ShooterConstants.hoodMax)/2;
    public Intake.ElementMode elementMode=Intake.ElementMode.DETECT;

    public static double curve(double input) {
        return Math.copySign(Math.pow(Math.abs(input),driveExpo), input);
    }

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        TimerEx timer = new TimerEx(120);
        Pose startPose = Robot.pose;
        Robot robot = new Robot(hardwareMap);
        if (startPose.x() == 0 && startPose.y() == 0) startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        robot.follower.setPose(startPose);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator=new GamepadEx(gamepad2);

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.IN))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new IntakeCommand(robot, Intake.IntakeState.OUT))
                .whenReleased(new IntakeCommand(robot, Intake.IntakeState.OFF));

        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new ParallelCommandGroup(
                        new TurretCommand(robot, Turret.TurretState.SET),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.shooter.setHoodPos(hoodPos)),
                                        new ShooterCommand(robot, Shooter.ShooterState.MANUAL)
                                ),
                                new ShooterCommand(robot, Shooter.ShooterState.AUTO),
                                () -> manualHood
                        )
                ))
                .whenHeld(new ShootCommand(robot));
//                .whenReleased(new ParallelCommandGroup(
//                        new TurretCommand(robot, Turret.TurretState.OFF),
//                        new ShooterCommand(robot, Shooter.ShooterState.OFF)
//                ));

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new InstantCommand(() -> manualHood = !manualHood));

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
                    curve(-driver.getLeftY()),
                    curve(-driver.getLeftX()),
                    curve(-driver.getRightX()));

            if (timer.isMoreThan(parkTime)) ManualDrive.driveOrHold(robot.follower, drivePowers); //look up driveorhold and look into it
            else {robot.follower.manual(drivePowers);}

            MyTelem.addData("Alliance", Robot.alliance);
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