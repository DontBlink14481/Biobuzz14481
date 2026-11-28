package org.firstinspires.ftc.teamcode.opmodes.prod;

import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.AutoFollowConstants.*;
import static org.firstinspires.ftc.teamcode.utils.constants.AutoConstants.SidPoses.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.FollowCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeForTimeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceSelection;
import org.firstinspires.ftc.teamcode.utils.DashPose;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Config
@Autonomous
public class SidThreeTip extends OpMode {

    TimerEx timer;

    Robot robot;

    AllianceSelection allianceSelection;
    Prompter prompter;

    CommandGroupBase auto;
    public static int shotCount = 4;
    static final double DEFAULT_POLLEN_X = 24.0;

    @Override
    public void init() {
        MyTelem.init(telemetry);

        auto = null;
        timer = new TimerEx(30);
        robot = new Robot(hardwareMap);
        robot.follower.setPose(routePose(start));

        RobotStates.shooterState = Shooter.ShooterState.AUTO;
        RobotStates.turretState = Turret.TurretState.SET;
        RobotStates.flowerIntakeState = FlowerIntake.FlowerIntakeState.OFF;

        robot.intake.setElementMode(Intake.ElementMode.DETECT);

        prompter = new Prompter(this);
        allianceSelection = new AllianceSelection(this, prompter);
        prompter.prompt(
                "pollenX",
                new ValuePrompt<>(
                        "Select an x for pollen line",
                        Double.class,
                        12.0,
                        36.0,
                        DEFAULT_POLLEN_X,
                        0.5
                )
        );
    }

    @Override
    public void init_loop() {
        allianceSelection.update();
        if (auto == null && prompter.isCompleted()) auto = buildAuto();

        robot.updatePose();
        robot.turret.periodic();
        robot.flowerIntake.periodic();

        MyTelem.addData("Pose", Robot.pose);
        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void start() {
        if (auto == null) auto = buildAuto();

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

    public static class Sid {
        public static Path path1() {
            return Paths.line(routePose(start), /*routePose(point1Control1), routePose(point1Control2),*/ routePose(point1)).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(routePose(point1, point1TurnFrom), routePose(point1, point1TurnMid))).until(1, Interpolator.linear(routePose(point1, point1TurnMid), routePose(point1, point1TurnTo))));
        }

        public static Path path2() {
            return Paths.curve(routePose(point1), routePose(point2Control1), routePose(point2Control2), routePose(point2)).heading(Interpolator.piecewise().until(0.2, Interpolator.linear(routePose(point2, point2TurnFrom), routePose(point2, point2TurnTo))).until(1, Interpolator.tangent));
        }

        public static Path path3() {
            return Paths.line(routePose(point2), routePose(point3)).constant(routePose(point3));
        }

        public static Path path4() {
            return Paths.line(routePose(point3), routePose(point4)).reverseTangent();
        }

        public static Path path5(Pose adjustedPoint5) {
            return Paths.curve(routePose(point4), routePose(point5Control1), adjustedPoint5).tangent();
        }

        public static Path path6(Pose adjustedPoint5) {
            return Paths.curve(adjustedPoint5, routePose(point6Control1), routePose(point6)).heading(Interpolator.piecewise().until(0.1, Interpolator.linear(routePose(point6, point6TurnFrom), routePose(point6))).until(1, Interpolator.constant(routePose(point6))));
        }

        public static Path path7() {
            return Paths.curve(routePose(point6), routePose(point7Control1), routePose(point7)).tangent();
        }

        public static Path path8() {
            return Paths.curve(routePose(point7), routePose(point8Control1), routePose(point8)).reverseTangent();
        }

        public static Path path9() {
            return Paths.curve(routePose(point8), routePose(point9Control1), routePose(point9)).tangent();
        }
    }

    CommandGroupBase buildAuto() {
        double pollenX = prompter.getOrDefault("pollenX", DEFAULT_POLLEN_X);
        Pose adjustedPoint5 = routePose(new DashPose(pollenX + 4, point5.y, point5.headingDeg));

        return new SequentialCommandGroup(
                new ShootCommand(robot, shotCount).afterDelay(firstShotWaitMs),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, Sid.path1(), cornerTimeoutMs),
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new FollowCommand(robot, Sid.path2(), longPathTimeoutMs).withFlowerOn(),
                new ShootCommand(robot, shotCount),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new SequentialCommandGroup(
                    new FollowCommand(robot, Sid.path3(), pathTimeoutMs),
                    new FollowCommand(robot, Sid.path4(), pathTimeoutMs),
                    new FollowCommand(robot, Sid.path3(), pathTimeoutMs)
                ).alongWith(new ShootCommand(robot, shotCount+1)),
                new FollowCommand(robot, Sid.path4(), pathTimeoutMs).withFlowerOff(),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, Sid.path5(adjustedPoint5), pathTimeoutMs),
                new IntakeCommand(robot, Intake.IntakeState.OFF),
                new FollowCommand(robot, Sid.path6(adjustedPoint5), longPathTimeoutMs),
                new ShootCommand(robot, shotCount+1),
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new FollowCommand(robot, Sid.path7(), pathTimeoutMs).withFlowerOn(),
                new IntakeForTimeCommand(robot, flowerIntakeMs),
                new FollowCommand(robot, Sid.path8(), pathTimeoutMs),
                new ShootCommand(robot, shotCount),
                new FollowCommand(robot, Sid.path9(), pathTimeoutMs).withFlowerOff()
        );
    }
}
