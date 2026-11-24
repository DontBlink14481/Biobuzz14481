package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.utils.constants.TurretConstants.targetAngleTesting;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.MathUtils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

public class Turret implements Subsystem {

    private final Servo turretServo1, turretServo2, turretServo3;
    private final AnalogInput turretEncoder2;

    private double targetAngleRad = 0;
    private double manualPosition = 0.5;
    private long aimUpdateCount = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, HardwareConstants.turret1);
        turretServo1.setDirection(Servo.Direction.REVERSE);

        turretServo2 = hardwareMap.get(Servo.class, HardwareConstants.turret2);
        turretServo2.setDirection(Servo.Direction.REVERSE);

        turretServo3 = hardwareMap.get(Servo.class, HardwareConstants.turret3);
        turretServo3.setDirection(Servo.Direction.FORWARD);

        turretEncoder2 = hardwareMap.get(AnalogInput.class, HardwareConstants.turretEncoder2);
    }

    @Override
    public void periodic() {
        setState(RobotStates.turretState);
    }

    public void setState(TurretState state) {
        if (state == null) return;
        if (RobotStates.turretState == TurretState.DISABLED && state != TurretState.DISABLED) return;
        RobotStates.turretState = state;
        calculateTurret();

        switch (state) {
            case MANUAL:
                setPosition(manualPosition);
                break;
            case CONTINUOUS:
                break;
            case SET:
                setAngle(targetAngleRad);
                break;
            case TUNING:
                setAngle(Math.toRadians(targetAngleTesting));
                break;
            case OFF:
                break;
            case DISABLED:
                setPosition(0.5);
                break;
        }
    }

    private void calculateTurret() {
        targetAngleRad = ShooterCalculator.calculate(Robot.pose, Robot.getTurretGoalPose()).turretAngleRad;
        aimUpdateCount++;
    }

    private void setAngle(double radians) {
        setPosition(TurretConstants.forwardPos + Math.toDegrees(radians) * TurretConstants.posPerDeg);
    }

    private void setPosition(double position) {
        double clippedPosition = Range.clip(position, 0, 1);
        turretServo1.setPosition(clippedPosition);
        turretServo2.setPosition(clippedPosition);
        turretServo3.setPosition(clippedPosition);
    }

    public void setManualPosition(double position) {
        manualPosition = Range.clip(position, 0, 1);
    }

    public double getTargetAngle() {
        return targetAngleRad;
    }

    public long getAimUpdateCount() {
        return aimUpdateCount;
    }

    public double getPosition() {
        return turretServo1.getPosition();
    }

    public double getServo2Position() {
        return turretServo2.getPosition();
    }

    public double getServo3Position() {
        return turretServo3.getPosition();
    }

    public double getEncoderVoltage() {
        if (turretEncoder2 == null) return Double.NaN;
        return turretEncoder2.getVoltage();
    }

    public double getAngle() {
        if (turretEncoder2 == null) return Double.NaN;
        double encoderRadians = MathUtils.normalizeRadians((TurretConstants.encoderZeroVoltage - getEncoderVoltage()) / turretEncoder2.getMaxVoltage() * 2 * Math.PI, false);
        return MathUtils.normalizeRadians(encoderRadians * TurretConstants.turretRadiansPerServoRadian(), false);
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
        MANUAL, CONTINUOUS, SET, OFF, DISABLED, TUNING
    }
}
