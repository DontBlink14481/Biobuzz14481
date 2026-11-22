package org.firstinspires.ftc.teamcode.robot.calculations;

public class CalculationResult {

    public double hoodAngleRad;
    public double turretAngleRad;
    public double rpm;
    public boolean valid;

    public CalculationResult(double hoodAngleRad, double turretAngleRad, double rpm, boolean valid) {
        this.hoodAngleRad = hoodAngleRad;
        this.turretAngleRad = turretAngleRad;
        this.rpm = rpm;
        this.valid = valid;
    }

    public static CalculationResult invalid(double turretAngleRad) {
        return new CalculationResult(Double.NaN, turretAngleRad, Double.NaN, false);
    }
}
