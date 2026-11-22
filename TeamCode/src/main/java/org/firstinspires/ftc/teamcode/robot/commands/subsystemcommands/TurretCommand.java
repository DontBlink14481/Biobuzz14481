package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

public class TurretCommand extends InstantCommand {
    public TurretCommand(Robot robot, Turret.TurretState state) {
        super(() -> RobotStates.turretState = state);
        addRequirements(robot.turret);
    }
}