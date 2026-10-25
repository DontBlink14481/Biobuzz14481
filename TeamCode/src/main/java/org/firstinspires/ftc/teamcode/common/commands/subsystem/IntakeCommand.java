package org.firstinspires.ftc.teamcode.common.commands.subsystem;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(Intake intake, Intake.IntakeState state) {
        super(() -> intake.set(state));
    }

    public IntakeCommand(Robot robot, Intake.IntakeState state) {
        super(() -> robot.intake.set(state));
    }
}
