package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;
import org.firstinspires.ftc.teamcode.utils.constants.TransferConstants;

public class NectarPauseCommand extends CommandBase {

    private final Robot robot;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private boolean wasNectar, paused;
    private Intake.IntakeState resumeState;

    public NectarPauseCommand(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        wasNectar = robot.transfer.inNectarMode();
        paused = false;
    }

    @Override //highkey some verified slop but im testing
    public void execute() {
        boolean nectar = robot.transfer.inNectarMode();
        if (wasNectar && !nectar && !paused) {
            paused = true;
            resumeState = RobotStates.intakeState;
            robot.intake.setState(Intake.IntakeState.OFF);
            pauseTimer.reset();
        }
        wasNectar = nectar;

        if (paused && pauseTimer.seconds() >= TransferConstants.nectarPauseS) {
            paused = false;
            robot.intake.setState(resumeState);
        }
    }
}
