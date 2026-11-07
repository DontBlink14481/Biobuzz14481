package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(Robot robot, Intake.IntakeState state) {
        super(() -> robot.intake.setState(state));
    }
}
