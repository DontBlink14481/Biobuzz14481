package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.BotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

import java.util.List;

@Config
public class Robot {

    public enum Alliance {
        RED, BLUE
    }

    public enum Goal {
        FRONT, BACK
    }

    public enum PedroState {
        ENABLED, DISABLED
    }

    public static Goal goal = Goal.FRONT;
    public static Pose pose = new Pose(0, 0, 0);

    public static Alliance alliance = Alliance.RED;

    public Follower follower;

    public List<LynxModule> hubs;
    public VoltageSensor voltageSensor;

    private boolean driveStopped = false;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    public Intake intake;
    public Transfer transfer;
    public Turret turret;
    public Shooter shooter;
    public Blocker blocker;

    public Robot(HardwareMap hardwareMap) {
        CommandScheduler.getInstance().reset();
        RobotStates.reset();

        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setNectarModeSupplier(transfer::inNectarMode);
        blocker = new Blocker(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        RobotConstants.voltage = voltageSensor.getVoltage();

        CommandScheduler.getInstance().registerSubsystem(intake, transfer, turret, shooter, blocker);
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
            driveStopped = false;
        } else if (!driveStopped) {
            follower.drivetrain.stop();
            driveStopped = true;
        }
        sendPose();

        CommandScheduler.getInstance().run();
        if (MyTelem.telemetry != null) MyTelem.update();
    }


    public static Pose getGoalPose() {
        return goal == Goal.FRONT ? BotConstants.frontGoal : BotConstants.backGoal;
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
