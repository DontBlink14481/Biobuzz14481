package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.ExodusPoses.*;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeForTimeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Autonomous
public class ExodusThreeTip extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;

    CommandGroupBase auto;

    public static long flowerIntakeMs = 2000;
    public static boolean sotm = false;

    @Override
    public void init() {
        MyTelem.init(telemetry);

        timer = new TimerEx(30);
        robot = new Robot(hardwareMap);
        robot.follower.setPose(start.pose());

        allianceSelection = new AllianceSelection(this);

        if (sotm) {

            SequentialCommandGroup paths = new SequentialCommandGroup(
                    new FollowPathCommand(robot.follower, Exodus.path1()),
                    new FollowPathCommand(robot.follower, Exodus.path2()),
                    new FollowPathCommand(robot.follower, Exodus.path3()),
                    new FollowPathCommand(robot.follower, Exodus.path4()),
                    new FollowPathCommand(robot.follower, Exodus.path5()),
                    new FollowPathCommand(robot.follower, Exodus.path6()),
                    new FollowPathCommand(robot.follower, Exodus.path7()),
                    new FollowPathCommand(robot.follower, Exodus.path8())
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
                new ShootCommand(robot),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, Exodus.path1()),
                new FollowPathCommand(robot.follower, Exodus.path2()),
                new ShootCommand(robot),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new ShootCommand(robot),
                new FollowPathCommand(robot.follower, Exodus.path3()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, Exodus.path4()),
                new FollowPathCommand(robot.follower, Exodus.path5()),
                new FollowPathCommand(robot.follower, Exodus.path6()),
                new ShootCommand(robot),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new FollowPathCommand(robot.follower, Exodus.path7()),
                new ShootCommand(robot),
                new FollowPathCommand(robot.follower, Exodus.path8())
        );
    }

    @Override
    public void init_loop() {
        allianceSelection.update();
        robot.update();

        MyTelem.addData("Pose", Robot.pose);
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void start() {
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
            return Paths.line(start.pose(), point1.pose()).constant(point1.pose());
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
