package org.firstinspires.ftc.teamcode.common.commands.subsystem;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;

public class BlockerCommand extends InstantCommand {
    public BlockerCommand(Blocker blocker, Blocker.BlockerState state) {
        super(() -> blocker.set(state));
    }

    public BlockerCommand(Robot robot, Blocker.BlockerState state) {
        this(robot.blocker, state);
    }
}
