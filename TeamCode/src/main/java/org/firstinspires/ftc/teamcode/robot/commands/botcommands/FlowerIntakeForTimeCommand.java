package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;

public class FlowerIntakeForTimeCommand extends SequentialCommandGroup {
    public FlowerIntakeForTimeCommand(Robot robot, long millis) {
        super(
                new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.ON),
                new WaitCommand(millis),
                new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.OFF)
        );
        addRequirements(robot.flowerIntake);
    }
}
