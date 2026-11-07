package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

public class IntakeTransferCommand extends ParallelCommandGroup {
    public IntakeTransferCommand(Robot robot, Intake.IntakeState state) {
        addCommands(
                new IntakeCommand(robot, state),
                new TransferCommand(robot, state == Intake.IntakeState.OFF ? Transfer.TransferState.OFF :
                        state == Intake.IntakeState.OUT ? Transfer.TransferState.OUT : Transfer.TransferState.IN)
        );
        addRequirements(robot.intake, robot.transfer);
    }
}
