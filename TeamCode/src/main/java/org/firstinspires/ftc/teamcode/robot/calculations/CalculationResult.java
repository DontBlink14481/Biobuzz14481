package org.firstinspires.ftc.teamcode.robot.calculations;

public class CalculationResult {

    public double hoodAngleRad;
    public double turretAngleRad;
    public boolean valid;

    public CalculationResult(double hoodAngleRad, double turretAngleRad, boolean valid) {
        this.hoodAngleRad = hoodAngleRad;
        this.turretAngleRad = turretAngleRad;
        this.valid = valid;
    }

    public static CalculationResult invalid(double turretAngleRad) {
        return new CalculationResult(Double.NaN, turretAngleRad, false);
    }
}
