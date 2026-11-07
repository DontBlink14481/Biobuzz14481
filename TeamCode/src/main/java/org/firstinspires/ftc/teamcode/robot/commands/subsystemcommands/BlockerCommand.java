package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;

public class BlockerCommand extends InstantCommand {
    public BlockerCommand(Robot robot, Blocker.BlockerState state) {
        super(() -> robot.blocker.setState(state));
    }
}
