package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.utils.constants.IntakeConstants;

public class NectarPauseCommand extends CommandBase {

    private final Robot robot;
    private final double feedMs;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private boolean wasNectar;
    private Pause pause;
    private double fedMs;

    public NectarPauseCommand(Robot robot) {
        this(robot, Double.POSITIVE_INFINITY);
    }

    public NectarPauseCommand(Robot robot, double feedMs) {
        this.robot = robot;
        this.feedMs = feedMs;
    }

    @Override
    public void initialize() {
        wasNectar = robot.intake.inNectarMode();
        pause = Pause.NONE;
        fedMs = 0;
    }

    @Override
    public void execute() {
        if (pause == Pause.NONE) fedMs += Robot.loopMs;

        boolean nectar = robot.intake.inNectarMode();
        if (nectar && !wasNectar) startPause(Pause.IN_BETWEEN);
        if (!nectar && wasNectar) startPause(Pause.PASSED);
        wasNectar = nectar;

        double t = pauseTimer.milliseconds();
        if (pause == Pause.IN_BETWEEN && t >= IntakeConstants.nectarPrepMs
                && (robot.shooter.atRPM() || t >= IntakeConstants.nectarPrepMaxMs)) {
            endPause();
        }
        if (pause == Pause.PASSED && t >= IntakeConstants.nectarPauseMs) {
            endPause();
        }
    }

    @Override
    public boolean isFinished() {
        return fedMs >= feedMs;
    }

    private void startPause(Pause next) {
        pause = next;
        robot.blocker.setState(Blocker.BlockerState.BLOCKED);
        pauseTimer.reset();
    }

    private void endPause() {
        pause = Pause.NONE;
        robot.blocker.setState(Blocker.BlockerState.UNBLOCKED);
    }

    private enum Pause {
        NONE, IN_BETWEEN, PASSED
    }
}
