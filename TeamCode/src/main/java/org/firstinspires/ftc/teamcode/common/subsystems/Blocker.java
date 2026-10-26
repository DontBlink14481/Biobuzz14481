package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.common.constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;

public class Blocker implements Subsystem {

    public Servo blocker;

    public BlockerState state;

    public Blocker(HardwareMap map) {
        blocker = map.get(Servo.class, HardwareConstants.blocker);

        state = BlockerState.UNBLOCKED;
    }

    @Override
    public void periodic() {
        set(state);
    }

    public void set(BlockerState state) {
        if (state == null) return;
        this.state = state;

        switch(state) {
            case BLOCKED:
                blocker.setPosition(BlockerConstants.blocked);
                break;
            case UNBLOCKED:
                blocker.setPosition(BlockerConstants.unblocked);
                break;
        }
    }

    public enum BlockerState {
        BLOCKED, UNBLOCKED
    }
}
