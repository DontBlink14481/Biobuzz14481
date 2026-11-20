package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

public class Intake implements Subsystem {

    private final DcMotorEx intakeMotor, transferMotor;

    private final Servo pinchServo;
    private final DigitalChannel leftBeamBreak, rightBeamBreak;

    private ElementMode elementMode = ElementMode.DETECT;

    public boolean leftTriggered, rightTriggered;

    private boolean inNectarMode = false;
    private long lastTriggerNanos = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.intake);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.transfer);
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pinchServo = hardwareMap.get(Servo.class, HardwareConstants.pinch);
        leftBeamBreak = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak1);
        rightBeamBreak = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak2);
    }

    @Override
    public void periodic() {
        updateNectarMode();
        setState(RobotStates.intakeState);
    }

    public void setState(IntakeState state) {
        if (state == null) return;
        if (RobotStates.intakeState == IntakeState.DISABLED && state != IntakeState.DISABLED) return;
        RobotStates.intakeState = state;
        if (state != IntakeState.DISABLED) {
            pinchServo.setPosition(inNectarMode ? IntakeConstants.nectarPos : IntakeConstants.pollenPos);
        }

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
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
            case DISABLED:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }

    private void updateNectarMode() {
        leftTriggered = leftBeamBreak.getState();
        rightTriggered = rightBeamBreak.getState();

        if (elementMode != ElementMode.DETECT) {
            inNectarMode = elementMode == ElementMode.NECTAR;
            return;
        }

        boolean triggered = leftTriggered;
        if (triggered) lastTriggerNanos = System.nanoTime();

        double elapsedMs = (System.nanoTime() - lastTriggerNanos) / 1e6;
        inNectarMode = elapsedMs < IntakeConstants.nectarDelayMs;
    }

    public boolean inNectarMode() {
        return inNectarMode;
    }

    public void setElementMode(ElementMode elementMode) {
        if (elementMode != null) this.elementMode = elementMode;
    }

    public enum ElementMode {
        DETECT, POLLEN, NECTAR
    }

    public enum IntakeState {
        IN, OUT, TRANSFER_ONLY, OFF, DISABLED
    }
}
