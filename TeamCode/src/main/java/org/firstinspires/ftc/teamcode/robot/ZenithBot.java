package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.IntakeTransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.botcommands.ShootCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.NamedCommands;
import org.horizon36596.zenith.solverslib.ZenithRobot;

@Config
public class ZenithBot implements ZenithRobot {

    public static double shootRPM = 1680;
    public static double hoodPos = 0.4; //need to tune

    private static final double CENTER = 70.75;

    private Robot robot;
    private final Subsystem drive = new Subsystem() {};

    @Override
    public void init(AutoFromFile opMode) {
        MyTelem.init(opMode.telemetry);
        robot = new Robot(opMode.hardwareMap);
    }

    @Override
    public void registerCommands(AutoFromFile opMode)
    {
        NamedCommands.register("setIntake", (args, ctx) -> new IntakeTransferCommand(robot,
                NamedCommands.argEnum(args, "setIntake", "state", Intake.IntakeState.class, Intake.IntakeState.OFF)));

        NamedCommands.register("score", (args, ctx) ->
                new ShootCommand(robot, NamedCommands.argInt(args, "score", "count")));

        NamedCommands.registerCondition("hopperFull", () -> false);
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
        return Robot.alliance.name();
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
        robot.shooter.setTargetRPM(shootRPM);
        robot.shooter.setHoodPos(hoodPos);
    }

    @Override
    public void periodic(AutoFromFile opMode) {
        robot.clearBulkCache();
        robot.follower.update();

        Pose pose = robot.follower.pose();
        Robot.pose = new Pose(pose.x() + CENTER, pose.y() + CENTER, pose.heading());
        Robot.sendPose();

        MyTelem.addData("Alliance", Robot.alliance);
        MyTelem.update();
    }

    @Override
    public void onStop(AutoFromFile opMode) {
        robot.stop();
    }
}