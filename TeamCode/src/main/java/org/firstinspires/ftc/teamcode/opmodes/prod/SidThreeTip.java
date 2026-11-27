package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.SidPoses.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeForTimeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Config
@Autonomous
public class SidThreeTip extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;

    CommandGroupBase auto;

    public static long flowerIntakeMs = 2000;
    public static long pathTimeoutMs = 3000, longPathTimeoutMs = 4500;
    public static long shotWaitMs = 500, firstShotWaitMs = 1500;
    public static int shotCount = 4;
    public static double wallShotToleranceInches = 3;

    @Override
    public void init() {
        MyTelem.init(telemetry);

        timer = new TimerEx(30);
        robot = new Robot(hardwareMap);
        robot.follower.setPose(routePose(start));

        RobotStates.shooterState = Shooter.ShooterState.AUTO;
        RobotStates.turretState = Turret.TurretState.SET;

        robot.intake.setElementMode(Intake.ElementMode.DETECT);

        allianceSelection = new AllianceSelection(this);

        Pose wallShotPose = routePose(point6);
        auto = new SequentialCommandGroup(
                new ShootCommand(robot, shotCount).afterDelay(firstShotWaitMs),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, ThreeSweep.path1(), pathTimeoutMs),
                new FollowCommand(robot, ThreeSweep.path2(), longPathTimeoutMs).withFlowerOn(),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, ThreeSweep.path3(), pathTimeoutMs),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, ThreeSweep.path4(), pathTimeoutMs).withFlowerOff()
                        .alongWith(new IntakeCommand(robot, Intake.IntakeState.IN)),
                new FollowCommand(robot, ThreeSweep.path5Approach(), pathTimeoutMs),
                new FollowCommand(robot, ThreeSweep.path5(), longPathTimeoutMs),
                new WaitCommand(shotWaitMs),
                new ConditionalCommand(
                        new ShootCommand(robot, shotCount),
                        Commands.none(),
                        () -> Math.hypot(Robot.pose.x() - wallShotPose.x(),
                                Robot.pose.y() - wallShotPose.y()) <= wallShotToleranceInches
                ),
                new FollowCommand(robot, ThreeSweep.path8(), pathTimeoutMs).withFlowerOn(),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new FollowCommand(robot, ThreeSweep.path9(), pathTimeoutMs).withFlowerOff(),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, ThreeSweep.path10(), pathTimeoutMs)
        );
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
            return Paths.curve(routePose(start), routePose(point1Control1), routePose(point1Control2), routePose(point1)).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(routePose(point1, point1TurnFrom), routePose(point1, point1TurnMid))).until(1, Interpolator.linear(routePose(point1, point1TurnMid), routePose(point1, point1TurnTo))));
        }

        public static Path path2() {
            return Paths.curve(routePose(point1), routePose(point2Control1), routePose(point2Control2), routePose(point2)).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(routePose(point2, point2TurnFrom), routePose(point2, point2TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.line(routePose(point2), routePose(point3)).constant(routePose(point3));
        }

        public static Path path4() {
            return Paths.curve(routePose(point3), routePose(point4Control1), routePose(point4)).heading(Interpolator.piecewise().until(0.4, Interpolator.linear(routePose(point4, point4TurnFrom), routePose(point4, point4TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path5Approach() {
            return Paths.curve(routePose(point4), routePose(point5Control1), routePose(point5)).tangent();
        }

        public static Path path5() {
            return Paths.line(routePose(point5), routePose(point6)).tangent();
        }

        public static Path path8() {
            return Paths.curve(routePose(point6), routePose(point9Control1), routePose(point9Control2), routePose(point9)).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(routePose(point9, point9TurnFrom), routePose(point9, point9TurnMid))).until(1, Interpolator.linear(routePose(point9, point9TurnMid), routePose(point9, point9TurnTo))));
        }

        public static Path path9() {
            return Paths.curve(routePose(point9), routePose(point10Control1), routePose(point10)).reverseTangent();
        }

        public static Path path10() {
            return Paths.curve(routePose(point10), routePose(point11Control1), routePose(point11)).tangent();
        }
    }
}
