package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.utils.constants.FlowerIntakeConstants;
import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

public class FlowerIntake implements Subsystem {

    private final Servo flowerIntakeServo;

    public FlowerIntake(HardwareMap hardwareMap) {
        flowerIntakeServo = hardwareMap.get(Servo.class, HardwareConstants.flowerIntake);
    }

    @Override
    public void periodic() {
        setState(RobotStates.flowerIntakeState);
    }

    public void setState(FlowerIntakeState state) {
        if (state == null) return;
        RobotStates.flowerIntakeState = state;

        switch (state) {
            case ON:
                flowerIntakeServo.setPosition(FlowerIntakeConstants.on);
                break;
            case OFF:
                flowerIntakeServo.setPosition(FlowerIntakeConstants.off);
                break;
        }
    }

    public enum FlowerIntakeState {
        ON, OFF
    }
}
