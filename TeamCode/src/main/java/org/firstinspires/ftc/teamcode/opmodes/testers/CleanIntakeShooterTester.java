package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
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


    @Override
    public void init() {
        MyTelem.init(telemetry);
//        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shooter.set(Shooter.ShooterState.MANUAL);
    }

    @Override
    public void loop() {
//        intake.set(intakeOn ? Intake.IntakeState.IN : Intake.IntakeState.OFF);
        transfer.set(transferOn ? Transfer.TransferState.IN : Transfer.TransferState.OFF);

        shooter.setTargetRPM(targetRPM);

        shooter.setHoodPos(hoodPos + (RobotConstants.inNectarMode ? nectarHoodOffset : 0));

        RobotConstants.manualNectar = manualNectar;

        //        intake.periodic();
        transfer.periodic();
        shooter.periodic();

        MyTelem.addData("target rpm", targetRPM);
        MyTelem.addData("rpm", shooter.getRPM());
//        MyTelem.addData("current draw 1", shooter.shooter1.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("current draw 2", shooter.shooter2.getCurrent(CurrentUnit.MILLIAMPS));
//        MyTelem.addData("total current draw", shooter.shooter1.getCurrent(CurrentUnit.MILLIAMPS) + shooter.shooter2.getCurrent(CurrentUnit.MILLIAMPS));
        MyTelem.addData("transfer power", transfer.transfer.getPower());
        MyTelem.addData("transfer amps", transfer.transfer.getCurrent(CurrentUnit.AMPS));
        MyTelem.addData("pinch pos", transfer.pinch.getPosition());
        MyTelem.addData("left", transfer.leftTriggered);
        MyTelem.addData("right", transfer.rightTriggered);
        MyTelem.addData("manual nectar", manualNectar);
        MyTelem.addData("in nectar mode", RobotConstants.inNectarMode);
        MyTelem.update();
    }
}
