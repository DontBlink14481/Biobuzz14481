package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.zones.ParkZone;
import org.firstinspires.ftc.teamcode.utils.zones.ShootingZone;

import java.util.List;

@Config
public class Robot {

    public enum Alliance {
        RED, BLUE
    }

    public enum Goal {
        RIGHT, LEFT
    }

    public enum PedroState {
        ENABLED, DISABLED
    }

    public static Goal goal = Goal.RIGHT;
    public static Pose pose = new Pose(0, 0, 0);

    public static Alliance alliance = Alliance.RED;
    public static PolygonZone robotZone = new PolygonZone(RobotConstants.robotWidth, RobotConstants.robotLength);

    public Follower follower;

    public List<LynxModule> hubs;
    public VoltageSensor voltageSensor;

    private boolean driveStopped = false;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    public Intake intake;
    public Turret turret;
    public Shooter shooter;
    public Blocker blocker;

    public Robot(HardwareMap hardwareMap) {
        CommandScheduler.getInstance().reset();
        RobotStates.reset();

        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setNectarModeSupplier(intake::inNectarMode);
        blocker = new Blocker(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        RobotConstants.voltage = voltageSensor.getVoltage();

        CommandScheduler.getInstance().registerSubsystem(intake, turret, shooter, blocker);
    }

    public void update() {
        clearBulkCache();

        if (voltageTimer.milliseconds() > 1000) {
            voltageTimer.reset();
            RobotConstants.voltage = voltageSensor.getVoltage();
        }

        if (RobotStates.pedroState == PedroState.ENABLED) {
            follower.update();
            pose = follower.pose();
            robotZone.setPosition(pose.x(), pose.y());
            robotZone.setRotation(pose.heading());
            driveStopped = false;
        } else if (!driveStopped) {
            follower.drivetrain.stop();
            driveStopped = true;
        }
        sendPose();
        updateGoal();

        CommandScheduler.getInstance().run();
        if (MyTelem.telemetry != null) MyTelem.update();
    }

    public static void updateGoal() {
        if (ShootingZone.right().contains(robotZone.getPosition())) goal = Goal.RIGHT;
        if (ShootingZone.left().contains(robotZone.getPosition())) goal = Goal.LEFT;
    }

    public static Pose getGoalPose() {
        return goal == Goal.RIGHT ? RobotConstants.rightGoal : RobotConstants.leftGoal;
    }

    public static boolean parked() {
        return robotZone.isInside(ParkZone.get());
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public static void sendPose() {
        MyTelem.addData("Pose x", pose.x());
        MyTelem.addData("Pose y", pose.y());
        MyTelem.addData("Pose heading", pose.heading());
    }

    public void stop() {
        CommandScheduler.getInstance().reset();
        clearBulkCache();
    }
}
