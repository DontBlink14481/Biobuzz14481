package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.common.shooting.ShooterCalculator;
import org.firstinspires.ftc.teamcode.common.shooting.ShooterResult;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public class Robot {

    public Follower follower;
    public Intake intake;
    public Transfer transfer;
    public Turret turret;
    public Shooter shooter;
    public Blocker blocker;

    List<LynxModule> hubs;

    public static Pose pose = new Pose(0, 0 ,0);

    public enum Goal {
        FRONT, BACK
    }
    public static Goal goal = Goal.FRONT;
    public static boolean auto = false;

    double selectedRPM = 0;

    public Robot(HardwareMap map) {
        this(map, false);
    }

    public Robot(HardwareMap map, boolean auto) {
        Robot.auto = auto;
        this.initHardware(map);
    }

    public void initHardware(HardwareMap map) {
        follower = Constants.create(map);

        intake = new Intake(map);
        transfer = new Transfer(map);
        turret = new Turret(map);
        shooter = new Shooter(map);
        blocker = new Blocker(map);

        hubs = map.getAll(LynxModule.class);

        for (LynxModule x : hubs) x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        CommandScheduler.getInstance().registerSubsystem(intake, transfer, turret, shooter, blocker);
    }

    public void periodic() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.update();
        pose = follower.pose();

        Pose goalPose = goal == Goal.FRONT ? RobotConstants.FRONT : RobotConstants.BACK;

        double distanceMeters = Math.hypot(goalPose.x() - pose.x(), goalPose.y() - pose.y()) * 0.0254;
        selectedRPM = ShooterCalculator.selectRPM(distanceMeters, selectedRPM);
        shooter.setTargetRPM(selectedRPM);

        ShooterCalculator.Element element = RobotConstants.inNectarMode ? ShooterCalculator.Element.NECTAR : ShooterCalculator.Element.POLLEN;
        ShooterResult result = ShooterCalculator.calculate(pose, goalPose, shooter.getRPM(), element);

        shooter.lastResult = result;

        CommandScheduler.getInstance().run();
    }

    public void stop() {
        CommandScheduler.getInstance().reset();
        for (LynxModule x : hubs) {
            x.clearBulkCache();
        }
    }

}
