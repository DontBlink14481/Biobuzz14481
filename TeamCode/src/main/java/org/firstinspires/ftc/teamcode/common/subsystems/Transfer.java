package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.common.constants.TransferConstants;

public class Transfer implements Subsystem {

    public DcMotorEx transfer;
    public CRServo dropdown;
    public Servo pinch;

    public DigitalChannel beambreak1;
    public DigitalChannel beambreak2;

    public TransferState state;

    public boolean leftTriggered;
    public boolean rightTriggered;

    public static double start = TransferConstants.nectarDelayMs;

    public Transfer(HardwareMap map) {
        transfer = map.get(DcMotorEx.class, HardwareConstants.transfer);

        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transfer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        dropdown = map.get(CRServo.class, HardwareConstants.dropdown);

        pinch = map.get(Servo.class, HardwareConstants.pinch);

        beambreak1 = map.get(DigitalChannel.class, HardwareConstants.beambreak1);
        beambreak2 = map.get(DigitalChannel.class, HardwareConstants.beambreak2);

        state = TransferState.OFF;
        RobotConstants.manualNectar = false;
    }

    @Override
    public void periodic() {
        set(state);

        setNectarMode();
    }

    public void set(TransferState state) {
        if (state == null) return;
        this.state = state;

        switch (state) {
            case IN:
                transfer.setPower(IntakeConstants.inPower);
                dropdown.setPower(TransferConstants.dropdownInPower);
                break;
            case OUT:
                transfer.setPower(IntakeConstants.outPower);
                dropdown.setPower(TransferConstants.dropdownOutPower);
                break;
            case OFF:
                transfer.setPower(IntakeConstants.offPower);
                dropdown.setPower(TransferConstants.dropdownOffPower);
                break;
        }

        pinch.setPosition(RobotConstants.inNectarMode ? TransferConstants.nectarPos : TransferConstants.pollenPos);

    }

    public void setNectarMode() {
        if (RobotConstants.manualNectar) return;

        leftTriggered = beambreak1.getState();
        rightTriggered = beambreak2.getState();

        boolean triggered = leftTriggered /*|| rightTriggered*/;

        if (triggered) start = System.nanoTime();

        double elapsed = (System.nanoTime() - start) / 1e6;

        RobotConstants.inNectarMode = elapsed < TransferConstants.nectarDelayMs;
    }

    public enum TransferState {
        IN, OUT, OFF
    }
}
