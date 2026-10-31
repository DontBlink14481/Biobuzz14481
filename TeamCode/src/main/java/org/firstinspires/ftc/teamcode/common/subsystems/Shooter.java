package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.common.shooting.ShooterResult;
import org.firstinspires.ftc.teamcode.utils.BlinkServo;

public class Shooter implements Subsystem {

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public BlinkServo hood;
    public Servo shift;

    public PIDFController controller;

    public static double targetRPM = 0;
    public static double hoodPos = 0;

    public ShooterResult lastResult = ShooterResult.invalid();

    public ShooterState state;

    public Shooter(HardwareMap map) {
        shooter1 = map.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooter2 = map.get(DcMotorEx.class, HardwareConstants.shooter2);

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hood = new BlinkServo(map, HardwareConstants.hood, ShooterConstants.hoodMinDeg, ShooterConstants.hoodMaxDeg, ShooterConstants.hoodMin, ShooterConstants.hoodMax)
                .setInverted(true);
        shift = map.get(Servo.class, HardwareConstants.shift);

        controller = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);

        state = ShooterState.AUTO;
    }

    @Override
    public void periodic() {
        set(state);

        controller.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        controller.setTolerance(ShooterConstants.rpmTolerance);
    }

    public void set(ShooterState state) {
        if (state == null) return;
        this.state = state;

        switch (state) {
            case MANUAL:

                hoodPos = Range.clip(hoodPos, ShooterConstants.hoodMin, ShooterConstants.hoodMax);
                hood.setPosition(hoodPos);

                break;
            case AUTO:
                if (lastResult.valid) {
                    hood.turnToAngle(lastResult.hoodAngleRad, AngleUnit.RADIANS);
                }
                break;
            case OFF:
                stop();
                break;
        }

        setPowerPID(targetRPM);

        shift.setPosition(RobotConstants.inNectarMode ? ShooterConstants.nectarPos : ShooterConstants.pollenPos);
    }

    public void setPowerBang(double targetRPM) {
        if (targetRPM <= 0) {
            stop();
            return;
        }

        double power = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void setPowerPID(double targetRPM) {
        if (targetRPM <= 0) {
            stop();
            return;
        }

        double power = Range.clip(controller.calculate(getRPM(), targetRPM), 0, 1);
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void setTargetRPM(double targetRPM) {
        Shooter.targetRPM = targetRPM;
    }

    public double getRPM() {
        return shooter1.getVelocity() * 60.0 / ShooterConstants.TICKS_PER_REV;
    }

    public boolean atRPM() {
//        return targetRPM > 0 && controller.atSetPoint();
        return targetRPM > 0 && Math.abs(Math.abs(targetRPM) - Math.abs(getRPM())) < ShooterConstants.rpmTolerance;
    }

    public void setHoodPos(double pos) {
        hoodPos = pos;
    }

    public void stop() {
        targetRPM = 0;
        controller.reset();
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    public enum ShooterState {
        MANUAL, AUTO, OFF
    }
}
