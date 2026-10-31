package org.firstinspires.ftc.teamcode.common.shooting;

public class ShooterResult {

    public double turretAngleRad;
    public double hoodAngleRad;
    public double rpm;
    public double flightTime;
    public boolean valid;

    public ShooterResult(double turretAngleRad, double hoodAngleRad, double rpm, double flightTime, boolean valid) {
        this.turretAngleRad = turretAngleRad;
        this.hoodAngleRad = hoodAngleRad;
        this.rpm = rpm;
        this.flightTime = flightTime;
        this.valid = valid;
    }

    public static ShooterResult invalid() {
        return new ShooterResult(Double.NaN, Double.NaN, Double.NaN, Double.NaN, false);
    }
}
