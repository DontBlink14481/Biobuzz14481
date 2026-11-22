package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class ShooterCommand extends InstantCommand {
    public ShooterCommand(Robot robot, Shooter.ShooterState state){
        super(() -> robot.shooter.setState(state));
        addRequirements(robot.shooter);

    }
}
