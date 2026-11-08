package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

public class Blocker implements Subsystem {

    private final Servo blockerServo;

    public Blocker(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, HardwareConstants.blocker);
    }

    @Override
    public void periodic() {
        setState(RobotStates.blockerState);
    }

    public void setState(BlockerState state) {
        if (state == null) return;
        if (RobotStates.blockerState == BlockerState.DISABLED && state != BlockerState.DISABLED) return;
        RobotStates.blockerState = state;

        switch (state) {
            case BLOCKED:
                blockerServo.setPosition(BlockerConstants.blocked);
                break;
            case UNBLOCKED:
                blockerServo.setPosition(BlockerConstants.unblocked);
                break;
            case DISABLED:
                break;
        }
    }

    public enum BlockerState {
        BLOCKED, UNBLOCKED, DISABLED
    }
}
