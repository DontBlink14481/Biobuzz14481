package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.ExodusPoses.*;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeForTimeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Autonomous
public class ExodusThreeTip extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;

    CommandGroupBase auto;

    public static long flowerIntakeMs = 1000;
    public static long pathTimeoutMs = 3000, longPathTimeoutMs = 4500;
    public static boolean sotm = false;
    public static long shotWaitMs = 500, firstShotWaitMs = 1500;
    public static int shotCount = 4;

    @Override
    public void init() {
        MyTelem.init(telemetry);

        timer = new TimerEx(30);
        robot = new Robot(hardwareMap);
        robot.follower.setPose(start.pose());

        RobotStates.shooterState = Shooter.ShooterState.AUTO;
        RobotStates.turretState = Turret.TurretState.SET;

        robot.intake.setElementMode(Intake.ElementMode.DETECT);

        allianceSelection = new AllianceSelection(this);

        if (sotm) {
            SequentialCommandGroup paths = new SequentialCommandGroup(
                    follow(Exodus.path1(), pathTimeoutMs),
                    follow(Exodus.path2(), longPathTimeoutMs),
                    follow(Exodus.path3(), pathTimeoutMs),
                    follow(Exodus.path4(), pathTimeoutMs),
                    follow(Exodus.path5(), longPathTimeoutMs),
                    follow(Exodus.path6(), pathTimeoutMs),
                    follow(Exodus.path7(), pathTimeoutMs),
                    follow(Exodus.path8(), pathTimeoutMs)
            );

            auto = new ParallelCommandGroup(
                    paths,
                    new ConditionalCommand(
                            new ShootCommand(robot),
                            Commands.none(),
                            Robot::inShootingZone
                    ).perpetually()
            );

            return;
        }

        auto = new SequentialCommandGroup(
                shoot(firstShotWaitMs),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(Exodus.path1(), pathTimeoutMs),
                followAndPickupFlowers(Exodus.path2(), longPathTimeoutMs),
                shoot(),
                followAndRetract(Exodus.path3(), pathTimeoutMs),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(Exodus.path4(), pathTimeoutMs),
                follow(Exodus.path5(), longPathTimeoutMs),
                shoot(),
                followAndPickupFlowers(Exodus.path6(), pathTimeoutMs),
                followAndRetract(Exodus.path7(), pathTimeoutMs),
                shoot(),
                follow(Exodus.path8(), pathTimeoutMs)
        );
    }

    private Command shoot() {
        return shoot(shotWaitMs);
    }

    private Command shoot(long waitMs) {
        return new SequentialCommandGroup(
                new WaitCommand(waitMs),
                new ShootCommand(robot, shotCount)
        );
    }

    private Command followAndPickupFlowers(Path path, long timeoutMs) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        follow(path, timeoutMs),
                        new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.ON)
                ),
                new IntakeForTimeCommand(robot, flowerIntakeMs)
        );
    }

    private Command followAndRetract(Path path, long TimeoutMs){
        return new ParallelCommandGroup(
                follow(path, TimeoutMs),
                new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.OFF)
        );
    }

    private Command follow(Path path, long timeoutMs) {
        return new FollowPathCommand(robot.follower, path).withTimeout(timeoutMs);
    }

    @Override
    public void init_loop() {
        allianceSelection.update();
        robot.updatePose();

        MyTelem.addData("Pose", Robot.pose);
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void start() {
        auto.schedule();
        timer.start();
    }

    @Override
    public void loop() {
        MyTelem.addData("Pose", Robot.pose);
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.addData("TIME LEFT", timer.getRemaining());

        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public static class Exodus {
        public static Path path1() {
            return Paths.curve(start.pose(), point1Control1.pose(), point1Control2.pose(), point1.pose()).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point1.pose(point1TurnFrom), point1.pose(point1TurnMid))).until(1, Interpolator.linear(point1.pose(point1TurnMid), point1.pose(point1TurnTo))));
        }

        public static Path path2() {
            return Paths.curve(point1.pose(), point2Control1.pose(), point2Control2.pose(), point2.pose()).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(point2.pose(point2TurnFrom), point2.pose(point2TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.line(point2.pose(), point3.pose()).reverseTangent();
        }

        public static Path path4() {
            return Paths.curve(point3.pose(), point4Control1.pose(), point4.pose()).tangent();
        }

        public static Path path5() {
            return Paths.line(point4.pose(), point5.pose()).heading(Interpolator.piecewise().until(0.1, Interpolator.linear(point5.pose(point5TurnFrom), point5.pose())).until(1, Interpolator.constant(point5.pose())));
        }

        public static Path path6() {
            return Paths.curve(point5.pose(), point6Control1.pose(), point6.pose()).tangent();
        }

        public static Path path7() {
            return Paths.curve(point6.pose(), point7Control1.pose(), point7.pose()).reverseTangent();
        }

        public static Path path8() {
            return Paths.curve(point7.pose(), point8Control1.pose(), point8.pose()).tangent();
        }
    }
}
