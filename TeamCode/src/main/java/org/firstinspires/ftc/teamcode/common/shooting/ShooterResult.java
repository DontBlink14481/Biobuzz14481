package org.firstinspires.ftc.teamcode.common.shooting;

public class ShooterResult {

    public double turretAngleRad;
    public double hoodAngleRad;
    public boolean valid;

    public ShooterResult(double turretAngleRad, double hoodAngleRad, boolean valid) {
        this.turretAngleRad = turretAngleRad;
        this.hoodAngleRad = hoodAngleRad;
        this.valid = valid;
    }

    public static ShooterResult invalid() {
        return new ShooterResult(0, 0, false);
    }
}
