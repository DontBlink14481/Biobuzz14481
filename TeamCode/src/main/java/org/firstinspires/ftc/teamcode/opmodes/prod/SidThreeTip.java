package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
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

public class SidThreeTip extends OpMode {

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

        allianceSelection = new AllianceSelection(this);

        if (sotm) {

            SequentialCommandGroup paths = new SequentialCommandGroup(
                    new FollowPathCommand(robot.follower, PathContainer.path1()),
                    new FollowPathCommand(robot.follower, PathContainer.path2()),
                    new FollowPathCommand(robot.follower, PathContainer.path3()),
                    new FollowPathCommand(robot.follower, PathContainer.path4()),
                    new FollowPathCommand(robot.follower, PathContainer.path5()),
                    new FollowPathCommand(robot.follower, PathContainer.path6()),
                    new FollowPathCommand(robot.follower, PathContainer.path7())
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
                new FollowPathCommand(robot.follower, PathContainer.path1()),
                new ShootCommand(robot),
                new FollowPathCommand(robot.follower, PathContainer.path2()),
                new ShootCommand(robot),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new ShootCommand(robot),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, PathContainer.path3()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, PathContainer.path4()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, PathContainer.path5()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowPathCommand(robot.follower, PathContainer.path6()),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new FollowPathCommand(robot.follower, PathContainer.path7()),
                new ShootCommand(robot)
        );
    }

    @Override
    public void init_loop() {
        allianceSelection.update();

        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void start() {
        timer.start();
    }

    @Override
    public void loop() {
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.addData("TIME LEFT", timer.getRemaining());

        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public static class PathContainer {

        private static final PoseFactory poseFactory = PoseFactory.degrees();

        private static final Pose start = poseFactory.of(40, 6.75, 90);
        private static final Pose path1 = poseFactory.of(8.75, 6.75, 180);
        private static final Pose point2 = poseFactory.of(32, 70, 90);
        private static final Pose point2Control1 = poseFactory.of(32, 6.75, 0);
        private static final Pose point2Segment1Heading = poseFactory.of(32, 70, 180);
        private static final Pose point2Segment2Start = poseFactory.of(32, 70, 180);
        private static final Pose point2Segment2End = poseFactory.of(32, 70, 90);
        private static final Pose point3 = poseFactory.of(42, 132, 90);
        private static final Pose point3Control1 = poseFactory.of(32, 101, 0);
        private static final Pose point3Control2 = poseFactory.of(42, 101, 0);
        private static final Pose point4 = poseFactory.of(62, 113, 270);
        private static final Pose point4Control1 = poseFactory.of(42, 113, 0);
        private static final Pose point4Control2 = poseFactory.of(62, 132, 0);
        private static final Pose point4Segment1Start = poseFactory.of(62, 113, 90);
        private static final Pose point4Segment1End = poseFactory.of(62, 113, 270);
        private static final Pose point5 = poseFactory.of(62, 9, -90);
        private static final Pose point6 = poseFactory.of(50, 9, 270);
        private static final Pose point6Control1 = poseFactory.of(62, 40, 0);
        private static final Pose point6Control2 = poseFactory.of(50, 40, 0);
        private static final Pose point7 = poseFactory.of(38, 9, 270);
        private static final Pose point7Control1 = poseFactory.of(50, 40, 0);
        private static final Pose point7Control2 = poseFactory.of(38, 40, 0);
        private static final Pose point8 = poseFactory.of(9, 42, 180);
        private static final Pose point8Control1 = poseFactory.of(38, 28, 0);
        private static final Pose point8Control2 = poseFactory.of(24, 42, 0);
        private static final Pose point8Segment1Start = poseFactory.of(9, 42, 270);
        private static final Pose point8Segment1End = poseFactory.of(9, 42, 150);
        private static final Pose point8Segment2Start = poseFactory.of(9, 42, 150);
        private static final Pose point8Segment2End = poseFactory.of(9, 42, 180);
        private static final Pose point9 = poseFactory.of(58, 22, 140);
        private static final Pose point9Control1 = poseFactory.of(34, 42, 0);

        public static Path path1() {
            return Paths.line(start, path1).constant(path1);
        }

        public static Path path2a() {
            return Paths.curve(path1, point2Control1, point2).heading(Interpolator.piecewise().until(0.05, Interpolator.constant(point2Segment1Heading)).until(0.5, Interpolator.linear(point2Segment2Start, point2Segment2End)).until(1, Interpolator.tangent));
        }

        public static Path path2b() {
            return Paths.curve(point2, point3Control1, point3Control2, point3).tangent();
        }

        public static Path path2() {
            return Paths.path(path2a(), path2b());
        }

        public static Path path3a() {
            return Paths.curve(point3, point4Control1, point4Control2, point4).heading(Interpolator.piecewise().until(1, Interpolator.linear(point4Segment1Start, point4Segment1End)));
        }

        public static Path path3b() {
            return Paths.line(point4, point5).tangent();
        }

        public static Path path3() {
            return Paths.path(path3a(), path3b());
        }

        public static Path path4() {
            return Paths.curve(point5, point6Control1, point6Control2, point6).constant(point6);
        }

        public static Path path5() {
            return Paths.curve(point6, point7Control1, point7Control2, point7).constant(point7);
        }

        public static Path path6() {
            return Paths.curve(point7, point8Control1, point8Control2, point8).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point8Segment1Start, point8Segment1End)).until(1, Interpolator.linear(point8Segment2Start, point8Segment2End)));
        }

        public static Path path7() {
            return Paths.curve(point8, point9Control1, point9).reverseTangent();
        }
    }
}
