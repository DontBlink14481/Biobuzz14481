package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

@Config
public class RobotStates {
    public static Robot.PedroState pedroState;
    public static Turret.TurretState turretState;
    public static Shooter.ShooterState shooterState;
    public static Intake.IntakeState intakeState;
    public static Transfer.TransferState transferState;
    public static Blocker.BlockerState blockerState;

    static {
        reset();
    }

    public static void reset() {
        pedroState = Robot.PedroState.ENABLED;
        turretState = Turret.TurretState.DISABLED;
        shooterState = Shooter.ShooterState.MANUAL;
        intakeState = Intake.IntakeState.OFF;
        transferState = Transfer.TransferState.OFF;
        blockerState = Blocker.BlockerState.BLOCKED;
    }
}
