package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.common.shooting.ShooterCalculator;
import org.firstinspires.ftc.teamcode.common.shooting.ShooterResult;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Clean Intake Shooter Tester")
public class CleanIntakeShooterTester extends OpMode {

//    Intake intake;
    Transfer transfer;
    Shooter shooter;

    public static boolean manualNectar = false;
    public static double targetRPM = 0;
    public static double hoodPos = 0.6;
    public static double nectarHoodOffset = 0.1;

    public static boolean intakeOn = false;
    public static boolean transferOn = false;

    public static double x = 0;
    public static double y = 0;
    public static double headingDeg = 0;
    public static Pose pose;

    public static Robot.Goal goal = Robot.Goal.BACK;
    public static Shooter.ShooterState shooterState = Shooter.ShooterState.AUTO;

    public static boolean autoRPM = true;
    double selectedRPM = 0;


    @Override
    public void init() {
        MyTelem.init(telemetry);
//        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

    }

    @Override
    public void loop() {
        RobotConstants.manualNectar = manualNectar;

//        intake.set(intakeOn ? Intake.IntakeState.IN : Intake.IntakeState.OFF);
        transfer.set(transferOn ? Transfer.TransferState.IN : Transfer.TransferState.OFF);

        //        intake.periodic();
        transfer.periodic();

        shooter.state = shooterState;

        pose = new Pose(x, y, Math.toRadians(headingDeg));

        Pose goalPose = goal == Robot.Goal.FRONT ? RobotConstants.FRONT : RobotConstants.BACK;

        double distanceMeters = Math.hypot(goalPose.x() - pose.x(), goalPose.y() - pose.y()) * 0.0254;
        selectedRPM = ShooterCalculator.selectRPM(distanceMeters, selectedRPM);
        shooter.setTargetRPM(selectedRPM);

        ShooterCalculator.Element element = RobotConstants.inNectarMode ? ShooterCalculator.Element.NECTAR : ShooterCalculator.Element.POLLEN;
        ShooterResult result = ShooterCalculator.calculate(pose, goalPose, shooter.getRPM(), element);

        shooter.lastResult = result;

        if (shooterState == Shooter.ShooterState.MANUAL) shooter.setHoodPos(hoodPos + (RobotConstants.inNectarMode ? nectarHoodOffset : 0));

        shooter.periodic();

        MyTelem.addData("distance meters", distanceMeters);
        MyTelem.addData("selected rpm", selectedRPM);
        MyTelem.addData("shot valid", result.valid);
        MyTelem.addData("hood angle deg", Math.toDegrees(result.hoodAngleRad));
        MyTelem.addData("target rpm", targetRPM);
        MyTelem.addData("rpm", shooter.getRPM());
//        MyTelem.addData("current draw 1", shooter.shooter1.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("current draw 2", shooter.shooter2.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("total current draw", shooter.shooter1.getCurrent(CurrentUnit.MILLIAMPS) + shooter.shooter2.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("transfer power", transfer.transfer.getPower());
//        MyTelem.addData("transfer amps", transfer.transfer.getCurrent(CurrentUnit.AMPS));
//        MyTelem.addData("pinch pos", transfer.pinch.getPosition());
        MyTelem.addData("left", transfer.leftTriggered);
        MyTelem.addData("right", transfer.rightTriggered);
        MyTelem.addData("manual nectar", manualNectar);
        MyTelem.addData("in nectar mode", RobotConstants.inNectarMode);
        MyTelem.update();
    }
}
