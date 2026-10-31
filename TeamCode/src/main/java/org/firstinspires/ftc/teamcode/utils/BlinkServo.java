package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BlinkServo extends ServoEx {

    private Servo servo;

    //always stored internally as radians
    private double maxAngle, minAngle;

    private double maxPosition = 1;
    private double minPosition = 0;
    private double cachingTolerance = 0.0001;
    private double lastPosition = Double.NaN;


    public BlinkServo(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit, double minPosition, double maxPosition) {
        servo = hw.get(Servo.class, servoName);

        this.minAngle = toRadians(minAngle, angleUnit);
        this.maxAngle = toRadians(maxAngle, angleUnit);
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    public void setMaxPosition(double maxPosition) {
        this.maxPosition = maxPosition;
    }

    public void setMinPosition(double minPosition) {
        this.minPosition = minPosition;
    }

    public double getMinPosition() {
        return minPosition;
    }

    public double getMaxPosition() {
        return maxPosition;
    }

    public BlinkServo(HardwareMap hw, String servoName, double minDegree, double maxDegree, double minPosition, double maxPosition) {
        this(hw, servoName, minDegree, maxDegree, AngleUnit.DEGREES, minPosition, maxPosition);
    }

    public void rotateByAngle(double angle, AngleUnit angleUnit) {
        angle = getAngle(angleUnit) + angle;
        turnToAngle(angle, angleUnit);
    }

    public void rotateByAngle(double degrees) {
        rotateByAngle(degrees, AngleUnit.DEGREES);
    }

    public void turnToAngle(double angle, AngleUnit angleUnit) {
        double angleRadians = Range.clip(toRadians(angle, angleUnit), minAngle, maxAngle);
        setPosition((angleRadians - minAngle) * (maxPosition - minPosition) / (getAngleRange(AngleUnit.RADIANS)) + minPosition);
    }

    public void turnToAngle(double degrees) {
        turnToAngle(degrees, AngleUnit.DEGREES);
    }

    public void rotateBy(double position) {
        position = getPosition() + position;
        setPosition(position);
    }

    public void setPosition(double position) {
        if (Double.isNaN(lastPosition) || Math.abs(position - lastPosition) > cachingTolerance) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    public void setRange(double min, double max, AngleUnit angleUnit) {
        this.minAngle = toRadians(min, angleUnit);
        this.maxAngle = toRadians(max, angleUnit);
    }

    public void setRange(double min, double max) {
        setRange(min, max, AngleUnit.DEGREES);
    }

    public BlinkServo setInverted(boolean isInverted) {
        servo.setDirection(isInverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        return this;
    }

    public boolean getInverted() {
        return Servo.Direction.REVERSE == servo.getDirection();
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public double getAngle(AngleUnit angleUnit) {
        double position = servo.getPosition();

        double t = (position - minPosition) / (maxPosition - minPosition);
        double angleRadians = minAngle + t * (maxAngle - minAngle);

        return fromRadians(angleRadians, angleUnit);
    }

    public double getAngle() {
        return getAngle(AngleUnit.DEGREES);
    }

    public double getAngleRange(AngleUnit angleUnit) {
        return fromRadians(maxAngle - minAngle, angleUnit);
    }

    public double getAngleRange() {
        return getAngleRange(AngleUnit.DEGREES);
    }

    @Override
    public void disable() {
        servo.close();
    }

    public ServoEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }

    @Override
    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "BlinkServo: " + port + "; " + controller;
    }

    private double toRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toRadians(angle) : angle;
    }
    private double fromRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(angle) : angle;
    }
}