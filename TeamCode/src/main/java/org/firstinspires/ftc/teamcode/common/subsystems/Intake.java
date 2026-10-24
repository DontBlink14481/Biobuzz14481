package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.IntakeConstants;

public class Intake implements Subsystem {

    DcMotorEx intake;

    IntakeState state;

    public Intake(HardwareMap map) {
        intake = map.get(DcMotorEx.class, HardwareConstants.intake);

        // TODO: verify motor direction
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic(){
        set(state);
    }

    public void set(IntakeState state) {
        if (state == null) return;
        this.state = state;

        switch (state) {
            case IN:
                intake.setPower(IntakeConstants.inPower);
                break;
            case OUT:
                intake.setPower(IntakeConstants.outPower);
                break;
            case OFF:
                intake.setPower(IntakeConstants.offPower);
                break;
        }
    }

    public enum IntakeState {
        IN, OUT, OFF
    }

}
