package org.firstinspires.ftc.teamcode.common.commands.subsystem;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;

public class TransferCommand extends InstantCommand {
    public TransferCommand(Transfer transfer, Transfer.TransferState state) {
        super(() -> transfer.set(state));
    }

    public TransferCommand(Robot robot, Transfer.TransferState state) {
        this(robot.transfer, state);
    }
}
