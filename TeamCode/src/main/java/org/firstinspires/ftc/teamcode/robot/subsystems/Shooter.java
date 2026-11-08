package org.firstinspires.ftc.teamcode.robot.subsystems;

import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.ShooterCalculator;
import org.firstinspires.ftc.teamcode.robot.calculations.CalculationResult;
import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

import java.util.Objects;
import java.util.function.BooleanSupplier;

public class Shooter implements Subsystem {

    private static final double TICKS_PER_REV = 28;

    private final DcMotorEx shooterMotor1, shooterMotor2;
    private final Servo hoodServo, shiftServo;
    private BooleanSupplier nectarModeSupplier = () -> false;

    private double targetRPM = 0;
    private double hoodPos = 0;

    public CalculationResult lastResult = CalculationResult.invalid(0);

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, HardwareConstants.shooter2);
        for (DcMotorEx motor : new DcMotorEx[] {shooterMotor1, shooterMotor2}) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        hoodServo = hardwareMap.get(Servo.class, HardwareConstants.hood);
        hoodServo.setDirection(Servo.Direction.REVERSE);
        shiftServo = hardwareMap.get(Servo.class, HardwareConstants.shift);
    }

    public void setNectarModeSupplier(BooleanSupplier nectarModeSupplier) {
        this.nectarModeSupplier = Objects.requireNonNull(nectarModeSupplier);
    }

    @Override
    public void periodic() {
        setState(RobotStates.shooterState);
    }

    public void setState(ShooterState state) {
        if (state == null) return;
        if (RobotStates.shooterState == ShooterState.DISABLED && state != ShooterState.DISABLED) return;
        RobotStates.shooterState = state;

        switch (state) {
            case MANUAL:
                hoodServo.setPosition(Range.clip(hoodPos, ShooterConstants.hoodMin, ShooterConstants.hoodMax));
                break;
            case AUTO:
                aimShooter();
                break;
            case OFF:
            case DISABLED:
                targetRPM = 0;
                break;
        }

        //bang bang
        double power;
        if (targetRPM <= 0) {
            power = 0;
        } else if (getRPM() < targetRPM) {
            power = 1;
        } else {
            double hold = ShooterConstants.kF * targetRPM / ShooterConstants.maxRPM;
            power = Range.clip(hold * RobotConstants.nominalVoltage / RobotConstants.voltage, 0, 1);
        }
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);

        if (state != ShooterState.DISABLED) {
            shiftServo.setPosition(nectarModeSupplier.getAsBoolean() ? ShooterConstants.nectarPos : ShooterConstants.pollenPos);
        }
    }

    private void aimShooter() {
        setTargetRPM(ShooterConstants.targetRPM);
        lastResult = ShooterCalculator.calculate(Robot.pose, Robot.getGoalPose());
        if (lastResult.valid) setHoodAngle(lastResult.hoodAngleRad);
    }

    private void setHoodAngle(double angleRad) {
        double min = Math.toRadians(ShooterConstants.hoodMinDeg);
        double max = Math.toRadians(ShooterConstants.hoodMaxDeg);
        hoodServo.setPosition(Range.scale(angleRad, min, max, ShooterConstants.hoodMin, ShooterConstants.hoodMax));
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = Math.min(targetRPM, ShooterConstants.maxRPM);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return Math.abs(shooterMotor1.getVelocity()) * 60.0 / TICKS_PER_REV;
    }

    public boolean atRPM() {
        return targetRPM > 0 && Math.abs(targetRPM - getRPM()) < ShooterConstants.rpmTolerance;
    }

    public void setHoodPos(double pos) {
        hoodPos = pos;
    }

    public enum ShooterState {
        MANUAL, AUTO, OFF, DISABLED
    }
}
