package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
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
        robot.follower.setPose(PoseFactory.degrees().of(56, 8, 90));

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
        private static final PoseFactory poseFactory = PoseFactory.degrees();

        private static final Pose start = poseFactory.of(40, 6.75, 90);
        private static final Pose path1 = poseFactory.of(8.75, 6.75, 180);
        private static final Pose point2 = poseFactory.of(42, 132, 90);
        private static final Pose point2Control1 = poseFactory.of(24, 6.75, 0);
        private static final Pose point2Control2 = poseFactory.of(42, 120, 0);
        private static final Pose point2Segment1Start = poseFactory.of(42, 132, 180);
        private static final Pose point2Segment1End = poseFactory.of(42, 132, 90);
        private static final Pose point3 = poseFactory.of(42, 122, 90);
        private static final Pose point4 = poseFactory.of(28, 134, 180);
        private static final Pose point4Control1 = poseFactory.of(42, 134, 0);
        private static final Pose point5 = poseFactory.of(28, 14, 90);
        private static final Pose point5Segment1Start = poseFactory.of(28, 14, 180);
        private static final Pose point5Segment1End = poseFactory.of(28, 14, 90);
        private static final Pose point5Segment2Heading = poseFactory.of(28, 14, 90);
        private static final Pose point6 = poseFactory.of(9, 42, 180);
        private static final Pose point6Control1 = poseFactory.of(28, 42, 0);
        private static final Pose point7 = poseFactory.of(30, 14, 90);
        private static final Pose point7Control1 = poseFactory.of(30, 42, 0);
        private static final Pose point8 = poseFactory.of(15, 92, 104.9951);
        private static final Pose point8Control1 = poseFactory.of(30, 36, 0);

        public static Path path1() {
            return Paths.line(start, path1).constant(path1);
        }

        public static Path path2() {
            return Paths.curve(path1, point2Control1, point2Control2, point2).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.line(point2, point3).reverseTangent();
        }

        public static Path path4() {
            return Paths.curve(point3, point4Control1, point4).tangent();
        }

        public static Path path5() {
            return Paths.line(point4, point5).heading(Interpolator.piecewise().until(0.1, Interpolator.linear(point5Segment1Start, point5Segment1End)).until(1, Interpolator.constant(point5Segment2Heading)));
        }

        public static Path path6() {
            return Paths.curve(point5, point6Control1, point6).tangent();
        }

        public static Path path7() {
            return Paths.curve(point6, point7Control1, point7).reverseTangent();
        }

        public static Path path8() {
            return Paths.curve(point7, point8Control1, point8).tangent();
        }
    }
}
