package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class IntakeForTimeCommand extends SequentialCommandGroup {
    public IntakeForTimeCommand(Robot robot, long millis) {
        super(
                new IntakeCommand(robot, Intake.IntakeState.IN),
                new WaitCommand(millis),
                new IntakeCommand(robot, Intake.IntakeState.OFF)
        );
        addRequirements(robot.intake);
    }
}
