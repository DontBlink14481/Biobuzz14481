package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.TransferConstants;

public class Transfer implements Subsystem {

    private final CRServo dropdownServo;
    private final Servo pinchServo;
    private final DigitalChannel leftBeamBreak, rightBeamBreak;

    private ElementMode elementMode = ElementMode.DETECT;

    public boolean leftTriggered, rightTriggered;

    private boolean inNectarMode = false;
    private long lastTriggerNanos = 0;

    public Transfer(HardwareMap hardwareMap) {
        dropdownServo = hardwareMap.get(CRServo.class, HardwareConstants.dropdown);
        pinchServo = hardwareMap.get(Servo.class, HardwareConstants.pinch);
        leftBeamBreak = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak1);
        rightBeamBreak = hardwareMap.get(DigitalChannel.class, HardwareConstants.beambreak2);
    }

    @Override
    public void periodic() {
        updateNectarMode();
        setState(RobotStates.transferState);
    }

    public void setState(TransferState state) {
        if (state == null) return;
        if (RobotStates.transferState == TransferState.DISABLED && state != TransferState.DISABLED) return;
        RobotStates.transferState = state;

        switch (state) {
            case IN:
                dropdownServo.setPower(TransferConstants.dropdownInPower);
                break;
            case OUT:
                dropdownServo.setPower(TransferConstants.dropdownOutPower);
                break;
            case OFF:
            case DISABLED:
                dropdownServo.setPower(0);
                break;
        }

        if (state != TransferState.DISABLED) {
            pinchServo.setPosition(inNectarMode ? TransferConstants.nectarPos : TransferConstants.pollenPos);
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
        inNectarMode = elapsedMs < TransferConstants.nectarDelayMs;
    }

    public boolean inNectarMode() {
        return inNectarMode;
    }

    public void setElementMode(ElementMode elementMode) {
        if (elementMode != null) this.elementMode = elementMode;
    }

    public enum TransferState {
        IN, OUT, OFF, DISABLED
    }

    public enum ElementMode {
        DETECT, POLLEN, NECTAR
    }
}
