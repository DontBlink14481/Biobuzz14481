package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;

public class FlowerIntakeCommand extends InstantCommand {
    public FlowerIntakeCommand(Robot robot, FlowerIntake.FlowerIntakeState state) {
        super(() -> robot.flowerIntake.setState(state));
        addRequirements(robot.flowerIntake);
    }
}
