package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.common.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.opmodes.competition.ChooseAlliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.NamedCommands;
import org.horizon36596.zenith.solverslib.ZenithRobot;

import java.util.List;

// for zenith.autos
public class DontBlinkRobot implements ZenithRobot {

    Robot robot;
    List<LynxModule> hubs;

    Subsystem drive = new Subsystem() {};

    @Override
    public void init(AutoFromFile opMode) {
        robot = new Robot(opMode.hardwareMap, true);
        hubs = opMode.hardwareMap.getAll(LynxModule.class);
    }

    @Override
    public void registerCommands(AutoFromFile opMode) {
        NamedCommands.register("setIntake", (args, ctx) -> {
            Intake.IntakeState state = NamedCommands.argEnum(args, "setIntake", "state",
                    Intake.IntakeState.class, Intake.IntakeState.OFF);
            return new InstantCommand(() -> setIntake(state));
        });

        NamedCommands.register("score", (args, ctx) ->
                new ShootCommand(robot, NamedCommands.argInt(args, "score", "count")));

        NamedCommands.registerCondition("hopperFull", () -> false);
    }

    void setIntake(Intake.IntakeState state) {
        robot.intake.set(state);

        if (state == Intake.IntakeState.IN) {
            robot.blocker.set(Blocker.BlockerState.BLOCKED);
            if (AutoConstants.transferWhileIntaking) robot.transfer.set(Transfer.TransferState.IN);
        } else if (state == Intake.IntakeState.OUT) {
            robot.transfer.set(Transfer.TransferState.OUT);
        } else {
            robot.transfer.set(Transfer.TransferState.OFF);
        }
    }

    @Override
    public Follower follower() {
        return robot.follower;
    }

    @Override
    public Subsystem drive() {
        return drive;
    }

    @Override
    public String runningAlliance() {
        return ChooseAlliance.alliance;
    }

    @Override
    public Pose2d mirror(Pose2d pose) {
        return new Pose2d(-pose.getX(), -pose.getY(), new Rotation2d(pose.getHeading() + Math.PI));
    }

    @Override
    public double defaultSpeedFraction() {
        return 0.8;
    }

    @Override
    public Path atSpeed(Path path, double speedFraction) {
        if (speedFraction >= 1) return path;
        return path.with(Constants.foresightConfig.maxPathSpeed.at(speedFraction));
    }

    @Override
    public void onStart(AutoFromFile opMode) {
        robot.shooter.set(Shooter.ShooterState.MANUAL);
        robot.shooter.setTargetRPM(AutoConstants.shooterRPM);
    }

    @Override
    public void periodic(AutoFromFile opMode) {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        robot.follower.update();
        Robot.pose = robot.follower.pose();
        Robot.sendPose();
    }
}