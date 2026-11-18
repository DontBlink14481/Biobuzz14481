package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.seattlesolvers.solverslib.util.MathUtils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class Turret implements Subsystem {

    private final CRServo turretServo1, turretServo2;
    private final AnalogInput turretEncoder1, turretEncoder2;
    private final PIDController pid = new PIDController(0, 0, 0);

    private double targetAngleRad = 0;
    public double manualPower = 0;
    private long aimUpdateCount = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, HardwareConstants.turret1);
        turretServo1.setDirection(CRServo.Direction.REVERSE);

        turretServo2 = hardwareMap.get(CRServo.class, HardwareConstants.turret2);
        turretServo2.setDirection(CRServo.Direction.REVERSE);

        turretEncoder1 = hardwareMap.get(AnalogInput.class, HardwareConstants.turretEncoder1);
        turretEncoder2 = hardwareMap.get(AnalogInput.class, HardwareConstants.turretEncoder2);
    }

    @Override
    public void periodic() {
        setState(RobotStates.turretState);
    }

    public void setState(TurretState state) {
        if (state == null) return;
        if (RobotStates.turretState == TurretState.DISABLED && state != TurretState.DISABLED) return;
        if (RobotStates.turretState != state) {
            pid.reset();
        }
        RobotStates.turretState = state;
        calculateTurret();

        switch (state) {
            case MANUAL:
                setPower(manualPower);
                break;
            case CONTINUOUS:
                if (turretEncoder1 == null) {
                    setPower(0);
                } else {
                    pid.setPID(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
                    double angleError = MathUtils.normalizeRadians(targetAngleRad - getAngle(), false);
                    setPower(Range.clip(pid.calculate(0, angleError), -1, 1));
                }
                break;
            case SET:
                setPosition(TurretConstants.forwardPos + Math.toDegrees(targetAngleRad) * TurretConstants.posPerDeg);
                break;
            case OFF:
            case DISABLED:
                setPower(0);
                break;
        }
    }

    private void calculateTurret() {
        targetAngleRad = ShooterCalculator.calculate(Robot.pose, Robot.getGoalPose()).turretAngleRad;
        aimUpdateCount++;
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

    public long getAimUpdateCount() {
        return aimUpdateCount;
    }

    public double getPower() {
        return turretServo1.getPower();
    }

    public double getServo2Power() {
        return turretServo2.getPower();
    }

    public double getEncoderVoltage() {
        if (turretEncoder1 == null) return Double.NaN;
        return turretEncoder1.getVoltage();
    }

    public double getAngle() {
        if (turretEncoder1 == null) return Double.NaN;
        return MathUtils.normalizeRadians((TurretConstants.encoderZeroVoltage - getEncoderVoltage()) / turretEncoder1.getMaxVoltage() * 2 * Math.PI, false);
    }

    public double getAngle(AngleUnit unit) {
        return unit.fromRadians(getAngle());
    }

    public double getEncoder2Voltage() {
        return turretEncoder2 == null ? Double.NaN : turretEncoder2.getVoltage();
    }

    public double getEncoder2Angle() {
        if (turretEncoder2 == null) return Double.NaN;
        return MathUtils.normalizeRadians((TurretConstants.encoder2ZeroVoltage - getEncoder2Voltage()) / turretEncoder2.getMaxVoltage() * 2 * Math.PI, false);
    }

    public double getEncoder2Angle(AngleUnit unit) {
        return unit.fromRadians(getEncoder2Angle());
    }

    public enum TurretState {
        MANUAL, CONTINUOUS, SET, OFF, DISABLED
    }
}
