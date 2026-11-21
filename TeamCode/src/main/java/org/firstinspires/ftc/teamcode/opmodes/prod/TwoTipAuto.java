package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.TwoTip;

@Config
@Autonomous(name = "Two Tip Auto")
public class TwoTipAuto extends OpMode {
    private ElapsedTime timer;
    private Robot robot;
    private Paths paths;
    private Command auto;

    private AllianceSelection allianceSelection;

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap);
        allianceSelection = new AllianceSelection(this);
        paths = new Paths(robot.follower);

        auto = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.Path1),
                        new FollowPathCommand(robot.follower, paths.Path2),
                        new FollowPathCommand(robot.follower, paths.Path3),
                        new FollowPathCommand(robot.follower, paths.Path4),
                        new FollowPathCommand(robot.follower, paths.Path5),
                        new FollowPathCommand(robot.follower, paths.Path6),
                        new FollowPathCommand(robot.follower, paths.Path7)
                ),
                new ConditionalCommand(
                        new ShootCommand(robot),
                        Commands.none(),
                        Robot::inShootingZone
                ).perpetually()
        ); // lowk this will only work if sotm is good
    }

    @Override
    public void init_loop() {
        allianceSelection.update();

        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void start() {
        timer.reset();
        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void loop() {
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.addData("TIMER", timer.seconds());

        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public static class Paths {
        public Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Pose startPose = TwoTip.START_POSE;
            Pose point1 = TwoTip.POINT_1;
            Pose point2 = TwoTip.POINT_2;
            Pose point3 = TwoTip.POINT_3;
            Pose point4 = TwoTip.POINT_4;
            Pose point5 = TwoTip.POINT_5;
            Pose point6 = TwoTip.POINT_6;
            Pose point7 = TwoTip.POINT_7;
            Pose path3Control = TwoTip.PATH3_CONTROL;
            Pose path4Control1 = TwoTip.PATH4_CONTROL_1;
            Pose path4Control2 = TwoTip.PATH4_CONTROL_2;
            Pose path5Control1 = TwoTip.PATH5_CONTROL_1;
            Pose path5Control2 = TwoTip.PATH5_CONTROL_2;
            Pose path6Control = TwoTip.PATH6_CONTROL;

            follower.setPose(startPose);

            Path1 = com.pedropathing.api.Paths.line(startPose, point1).linear(startPose, point1);
            Path2 = com.pedropathing.api.Paths.line(point1, point2).linear(point1, point2);
            Path3 = com.pedropathing.api.Paths.curve(point2, path3Control, point3).linear(point2, point3);
            Path4 = com.pedropathing.api.Paths.curve(point3, path4Control1, path4Control2, point4).linear(point3, point4);
            Path5 = com.pedropathing.api.Paths.curve(point4, path5Control1, path5Control2, point5).linear(point4, point5);
            Path6 = com.pedropathing.api.Paths.curve(point5, path6Control, point6).linear(point5, point6);
            Path7 = com.pedropathing.api.Paths.line(point6, point7).linear(point6, point7);
        }
    }
}