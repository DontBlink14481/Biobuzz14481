package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

public class TransferCommand extends InstantCommand {
    public TransferCommand(Robot robot, Transfer.TransferState state) {
        super(() -> robot.transfer.setState(state));
    }
}
