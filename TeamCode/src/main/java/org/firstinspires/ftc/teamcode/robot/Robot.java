package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    public enum Goal {
        FRONT, BACK
    }

    public enum PedroState {
        ENABLED, DISABLED
    }

    public static Goal goal = Goal.FRONT;
    public static Pose pose = new Pose(0, 0, 0);

    public Follower follower;

    public DcMotorEx intakeMotor;
    public DcMotorEx transferMotor;
    public CRServo dropdownServo;
    public Servo pinchServo;
    public DigitalChannel leftBeamBreak, rightBeamBreak;
    public DcMotorEx shooterMotor1, shooterMotor2;
    public Servo hoodServo, shiftServo;
    public CRServo turretServo1, turretServo2;
    public DcMotorEx turretEncoder;
    public Servo blockerServo;

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

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        dropdownServo = hardwareMap.get(CRServo.class, "dropdown");
        pinchServo = hardwareMap.get(Servo.class, "pinch");
        leftBeamBreak = hardwareMap.get(DigitalChannel.class, "left");
        rightBeamBreak = hardwareMap.get(DigitalChannel.class, "right");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        for (DcMotorEx motor : new DcMotorEx[] {shooterMotor1, shooterMotor2}) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        shiftServo = hardwareMap.get(Servo.class, "shift");

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.tryGet(DcMotorEx.class, "turretEncoder");
        if (turretEncoder != null) {
            turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        blockerServo = hardwareMap.get(Servo.class, "blocker");

        intake = new Intake(intakeMotor, transferMotor);
        transfer = new Transfer(dropdownServo, pinchServo, leftBeamBreak, rightBeamBreak);
        turret = new Turret(turretServo1, turretServo2, turretEncoder);
        shooter = new Shooter(shooterMotor1, shooterMotor2, hoodServo, shiftServo, transfer);
        blocker = new Blocker(blockerServo);

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
