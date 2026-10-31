package org.firstinspires.ftc.teamcode.common.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.common.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;

// Shoots a set number of pieces: wait for shooter speed, open the blocker, run the transfer for that many pieces,
// then close up. The shooter keeps spinning for the next volley. Nothing counts pieces leaving, so the feed is timed.
public class ShootCommand extends CommandBase {

    Robot robot;
    int count;

    ElapsedTime timer = new ElapsedTime();
    boolean feeding;
    boolean done;

    public ShootCommand(Robot robot, int count) {
        this.robot = robot;
        this.count = count;
        addRequirements(robot.shooter, robot.transfer, robot.blocker);
    }

    @Override
    public void initialize() {
        robot.shooter.set(Shooter.ShooterState.MANUAL);
        robot.shooter.setTargetRPM(AutoConstants.shooterRPM);
        robot.blocker.set(Blocker.BlockerState.BLOCKED);
        robot.transfer.set(Transfer.TransferState.OFF);

        feeding = false;
        done = false;
        timer.reset();
    }

    @Override
    public void execute() {
        double hood = AutoConstants.hoodPos;
        if (RobotConstants.inNectarMode) hood += AutoConstants.nectarHoodOffset;
        robot.shooter.setHoodPos(hood);

        if (!feeding) {
            boolean upToSpeed = robot.shooter.getRPM() >= AutoConstants.shooterRPM - AutoConstants.rpmWindow;
            if (upToSpeed || timer.seconds() > AutoConstants.spinUpTimeoutS) {
                robot.blocker.set(Blocker.BlockerState.UNBLOCKED);
                robot.transfer.set(Transfer.TransferState.IN);
                feeding = true;
                timer.reset();
            }
        } else if (timer.seconds() >= count * AutoConstants.feedSecondsPerPiece) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        robot.transfer.set(Transfer.TransferState.OFF);
        robot.blocker.set(Blocker.BlockerState.BLOCKED);
    }
}
