package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.SidPoses.*;

import com.pedropathing.api.Paths;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
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

@Config
@Autonomous
public class SidThreeTip extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;

    CommandGroupBase auto;

    public static long flowerIntakeMs = 2000;
    public static boolean sotm = true;
    public static long pathTimeoutMs = 3000;

    @Override
    public void init() {
        MyTelem.init(telemetry);

        timer = new TimerEx(30);
        robot = new Robot(hardwareMap);
        robot.follower.setPose(start.pose());

        allianceSelection = new AllianceSelection(this);

        if (sotm) {

            SequentialCommandGroup paths = new SequentialCommandGroup(
                    follow(ThreeSweep.path1()),
                    follow(ThreeSweep.path2()),
                    follow(ThreeSweep.path3()),
                    follow(ThreeSweep.path4()),
                    follow(ThreeSweep.path5()),
                    follow(ThreeSweep.path6()),
                    follow(ThreeSweep.path7()),
                    follow(ThreeSweep.path8()),
                    follow(ThreeSweep.path9())
            );

            auto = new ParallelCommandGroup(
                    paths/*,
                    new ConditionalCommand(
                            new ShootCommand(robot),
                            Commands.none(),
                            Robot::inShootingZone
                    ).perpetually()*/
            );

            return;
        }

        auto = new SequentialCommandGroup(
                new ShootCommand(robot),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(ThreeSweep.path1()),
                follow(ThreeSweep.path2()),
                new ShootCommand(robot),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new ShootCommand(robot),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(ThreeSweep.path3()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(ThreeSweep.path4()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(ThreeSweep.path5()),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                follow(ThreeSweep.path6()),
                new ShootCommand(robot),
                follow(ThreeSweep.path7()),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                follow(ThreeSweep.path8()),
                new ShootCommand(robot),
                follow(ThreeSweep.path9())
        );
    }

    private Command follow(Path path) {
        return new FollowPathCommand(robot.follower, path).withTimeout(pathTimeoutMs);
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

    public static class ThreeSweep {
        public static Path path1() {
            return Paths.line(Robot.pose, point1.pose()).constant(point1.pose());
        }

        public static Path path2() {
            return Paths.curve(point1.pose(), point2Control1.pose(), point2Control2.pose(), point2.pose()).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(point2.pose(point2TurnFrom), point2.pose(point2TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.curve(point2.pose(), point3Control1.pose(), point3.pose()).heading(Interpolator.piecewise().until(0.4, Interpolator.linear(point3.pose(point3TurnFrom), point3.pose(point3TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path4() {
            return Paths.path(
                    Paths.curve(point3.pose(), point4Control1.pose(), point4.pose()).tangent(),
                    Paths.line(point4.pose(), point5.pose()).tangent());
        }

        public static Path path5() {
            return Paths.curve(point5.pose(), point6Control1.pose(), point6Control2.pose(), point6.pose()).constant(point6.pose());
        }

        public static Path path6() {
            return Paths.curve(point6.pose(), point7Control1.pose(), point7Control2.pose(), point7.pose()).constant(point7.pose());
        }

        public static Path path7() {
            return Paths.curve(point7.pose(), point8Control1.pose(), point8Control2.pose(), point8.pose()).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point8.pose(point8TurnFrom), point8.pose(point8TurnMid))).until(1, Interpolator.linear(point8.pose(point8TurnMid), point8.pose(point8TurnTo))));
        }

        public static Path path8() {
            return Paths.curve(point8.pose(), point9Control1.pose(), point9.pose()).reverseTangent();
        }

        public static Path path9() {
            return Paths.curve(point9.pose(), point10Control1.pose(), point10.pose()).tangent();
        }
    }
}
