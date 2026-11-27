package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;

public class FollowCommand extends SequentialCommandGroup {
    private final Robot robot;

    public FollowCommand(Robot robot, Path path, long timeoutMs) {
        super(new FollowPathCommand(robot.follower, path).withTimeout(timeoutMs));
        this.robot = robot;
    }

    public Command withFlowerOn() {
        return alongWith(new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.ON));
    }

    public Command withFlowerOff() {
        return alongWith(new FlowerIntakeCommand(robot, FlowerIntake.FlowerIntakeState.OFF));
    }
}
