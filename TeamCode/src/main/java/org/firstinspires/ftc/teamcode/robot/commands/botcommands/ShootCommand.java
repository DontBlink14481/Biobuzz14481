package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.BlockerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class ShootCommand extends SequentialCommandGroup {

    private final Robot robot;

    public ShootCommand(Robot robot) {
        this(robot, Double.POSITIVE_INFINITY);
    }

    public ShootCommand(Robot robot, int count) {
        this(robot, count * IntakeConstants.feedPerPiece);
    }

    public Command afterDelay(long waitMs) {
        return new SequentialCommandGroup(
                new WaitCommand(waitMs),
                this
        );
    }

    private ShootCommand(Robot robot, double feedMs) {
        this.robot = robot;
        addCommands(
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.BLOCKED)
                ),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.shooter::atRPM),
                        new WaitCommand(Math.round(ShooterConstants.spinUpTimeout))
                ),
                new ParallelCommandGroup(
                        new BlockerCommand(robot, Blocker.BlockerState.UNBLOCKED),
                        new IntakeCommand(robot, Intake.IntakeState.IN)
                ),
                new NectarPauseCommand(robot, feedMs)
        );
        addRequirements(robot.intake, robot.blocker);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.blocker.setState(Blocker.BlockerState.BLOCKED);
    }
}
