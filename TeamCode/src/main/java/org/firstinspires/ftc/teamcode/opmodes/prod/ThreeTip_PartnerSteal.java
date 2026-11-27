package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.PartnerStealPoses.*;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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

@Autonomous
public class ThreeTip_PartnerSteal extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;

    CommandGroupBase auto;

    public static long flowerIntakeMs = 1000;
    public static long pathTimeoutMs = 3000, longPathTimeoutMs = 4500;
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

        auto = new SequentialCommandGroup(
                new ShootCommand(robot, shotCount).afterDelay(firstShotWaitMs),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, PartnerSteal.path1(), pathTimeoutMs),
                new FollowCommand(robot, PartnerSteal.path2(), longPathTimeoutMs).withFlowerOn(),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, PartnerSteal.path3(), pathTimeoutMs),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, PartnerSteal.path4(), pathTimeoutMs).withFlowerOff(),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, PartnerSteal.path5(), pathTimeoutMs),
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new FollowCommand(robot, PartnerSteal.path6(), longPathTimeoutMs),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, PartnerSteal.path7(), pathTimeoutMs).withFlowerOn(),
                new FollowCommand(robot, PartnerSteal.path8(), pathTimeoutMs).withFlowerOff(),
                new ShootCommand(robot, shotCount).afterDelay(shotWaitMs),
                new FollowCommand(robot, PartnerSteal.path9(), pathTimeoutMs)
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

    public static class PartnerSteal {
        public static Path path1() {
            return Paths.curve(start.pose(), point1Control1.pose(), point1Control2.pose(), point1.pose()).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point1.pose(point1TurnFrom), point1.pose(point1TurnMid))).until(1, Interpolator.linear(point1.pose(point1TurnMid), point1.pose(point1TurnTo))));
        }

        public static Path path2() {
            return Paths.curve(point1.pose(), point2Control1.pose(), point2Control2.pose(), point2.pose()).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(point2.pose(point2TurnFrom), point2.pose(point2TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.line(point2.pose(), point4.pose()).constant(point3.pose());
        }

        public static Path path4() {
            return Paths.line(point3.pose(), point4.pose()).reverseTangent();
        }

        public static Path path5() {
            return Paths.curve(point4.pose(), point5Control1.pose(), point5.pose()).tangent();
        }

        public static Path path6() {
            return Paths.line(point5.pose(), point6.pose()).heading(Interpolator.piecewise().until(0.1, Interpolator.linear(point6.pose(point6TurnFrom), point6.pose())).until(1, Interpolator.constant(point6.pose())));
        }

        public static Path path7() {
            return Paths.curve(point6.pose(), point7Control1.pose(), point7.pose()).tangent();
        }

        public static Path path8() {
            return Paths.curve(point7.pose(), point8Control1.pose(), point8.pose()).reverseTangent();
        }

        public static Path path9() {
            return Paths.curve(point8.pose(), point9Control1.pose(), point9.pose()).tangent();
        }
    }
}
