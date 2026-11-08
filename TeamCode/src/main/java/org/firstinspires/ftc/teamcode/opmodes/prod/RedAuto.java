package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.Red2Tip;

@Autonomous(name = "Red Auto", group = "Prod")
@Config
public class RedAuto extends OpMode {
    private ElapsedTime timer;
    private Robot robot;
    private RedAutoPaths paths;
    private SequentialCommandGroup auto;
    private final String color;

    public RedAuto() {
        this("RED");
    }

    protected RedAuto(String color) {
        this.color = color;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        MyTelem.init(telemetry);
        robot = new Robot(hardwareMap);
        paths = new RedAutoPaths(robot.follower, color);

        auto = new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.Path1),
                new FollowPathCommand(robot.follower, paths.Path2),
                new FollowPathCommand(robot.follower, paths.Path3),
                new FollowPathCommand(robot.follower, paths.Path4),
                new FollowPathCommand(robot.follower, paths.Path5),
                new FollowPathCommand(robot.follower, paths.Path6),
                new FollowPathCommand(robot.follower, paths.Path7)
        );
    }

    @Override
    public void start() {
        timer.reset();
        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void loop() {
        MyTelem.addData("TIMER", timer.seconds());
        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public static class RedAutoPaths {
        public Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public RedAutoPaths(Follower follower, String color) {
            Pose startPose = Red2Tip.mirror(Red2Tip.START_POSE, color);
            Pose point1 = Red2Tip.mirror(Red2Tip.POINT_1, color);
            Pose point2 = Red2Tip.mirror(Red2Tip.POINT_2, color);
            Pose point3 = Red2Tip.mirror(Red2Tip.POINT_3, color);
            Pose point4 = Red2Tip.mirror(Red2Tip.POINT_4, color);
            Pose point5 = Red2Tip.mirror(Red2Tip.POINT_5, color);
            Pose point6 = Red2Tip.mirror(Red2Tip.POINT_6, color);
            Pose point7 = Red2Tip.mirror(Red2Tip.POINT_7, color);
            Pose path3Control = Red2Tip.mirror(Red2Tip.PATH3_CONTROL, color);
            Pose path4Control1 = Red2Tip.mirror(Red2Tip.PATH4_CONTROL_1, color);
            Pose path4Control2 = Red2Tip.mirror(Red2Tip.PATH4_CONTROL_2, color);
            Pose path5Control1 = Red2Tip.mirror(Red2Tip.PATH5_CONTROL_1, color);
            Pose path5Control2 = Red2Tip.mirror(Red2Tip.PATH5_CONTROL_2, color);
            Pose path6Control = Red2Tip.mirror(Red2Tip.PATH6_CONTROL, color);

            follower.setPose(startPose);

            Path1 = Paths.line(startPose, point1).linear(startPose, point1);
            Path2 = Paths.line(point1, point2).linear(point1, point2);
            Path3 = Paths.curve(point2, path3Control, point3).linear(point2, point3);
            Path4 = Paths.curve(point3, path4Control1, path4Control2, point4).linear(point3, point4);
            Path5 = Paths.curve(point4, path5Control1, path5Control2, point5).linear(point4, point5);
            Path6 = Paths.curve(point5, path6Control, point6).linear(point5, point6);
            Path7 = Paths.line(point6, point7).linear(point6, point7);
        }
    }
}