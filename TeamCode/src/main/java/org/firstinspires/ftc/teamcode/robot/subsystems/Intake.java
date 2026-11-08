package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

public class Intake implements Subsystem {

    private final DcMotorEx intakeMotor, transferMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.intake);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.transfer);
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        setState(RobotStates.intakeState);
    }

    public void setState(IntakeState state) {
        if (state == null) return;
        if (RobotStates.intakeState == IntakeState.DISABLED && state != IntakeState.DISABLED) return;
        RobotStates.intakeState = state;

        switch (state) {
            case IN:
                intakeMotor.setPower(IntakeConstants.inPower);
                transferMotor.setPower(IntakeConstants.transferInPower);
                break;
            case OUT:
                intakeMotor.setPower(IntakeConstants.outPower);
                transferMotor.setPower(IntakeConstants.transferOutPower);
                break;
            case TRANSFER_ONLY:
                intakeMotor.setPower(0);
                transferMotor.setPower(IntakeConstants.transferInPower);
                break;
            case OFF:
            case DISABLED:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }

    public enum IntakeState {
        IN, OUT, TRANSFER_ONLY, OFF, DISABLED
    }
}
