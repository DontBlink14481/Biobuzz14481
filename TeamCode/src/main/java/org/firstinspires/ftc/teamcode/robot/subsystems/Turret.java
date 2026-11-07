package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class Turret implements Subsystem {

    private final CRServo turretServo1, turretServo2;
    private final DcMotorEx turretEncoder;
    private final PIDController pid = new PIDController(0, 0, 0);

    private double targetAngleRad = 0;
    private double manualPower = 0;

    public Turret(CRServo turretServo1, CRServo turretServo2, DcMotorEx turretEncoder) {
        this.turretServo1 = turretServo1;
        this.turretServo2 = turretServo2;
        this.turretEncoder = turretEncoder;
    }

    @Override
    public void periodic() {
        setState(RobotStates.turretState);
    }

    public void setState(TurretState state) {
        if (state == null) return;
        if (RobotStates.turretState == TurretState.DISABLED && state != TurretState.DISABLED) return;
        RobotStates.turretState = state;

        switch (state) {
            case MANUAL:
                setPower(manualPower);
                break;
            case CONTINUOUS:
                aimTurret();
                if (turretEncoder == null) {
                    setPower(0);
                } else {
                    pid.setPID(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
                    setPower(pid.calculate(getAngle(), targetAngleRad));
                }
                break;
            case SET:
                aimTurret();
                setPosition(TurretConstants.forwardPos + Math.toDegrees(targetAngleRad) * TurretConstants.posPerDeg);
                break;
            case OFF:
            case DISABLED:
                setPower(0);
                break;
        }
    }

    private void aimTurret() {
        targetAngleRad = ShooterCalculator.calculate(Robot.pose, Robot.getGoalPose()).turretAngleRad;
    }

    private void setPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(power);
    }

    private void setPosition(double position) {
        setPower(2 * Range.clip(position, 0, 1) - 1);
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public double getTargetAngle() {
        return targetAngleRad;
    }

    public double getAngle() {
        if (turretEncoder == null) return Double.NaN;
        return turretEncoder.getCurrentPosition() / TurretConstants.ticksPerRev * 2 * Math.PI;
    }

    public enum TurretState {
        MANUAL, CONTINUOUS, SET, OFF, DISABLED
    }
}
