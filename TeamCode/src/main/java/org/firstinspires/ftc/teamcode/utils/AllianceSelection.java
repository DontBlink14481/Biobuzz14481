package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.Prompt;
import com.skeletonarmy.marrow.internal.Button;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Robot.Alliance;

public class AllianceSelection {
    private final OpMode opMode;
    private final Prompter prompter;

    public AllianceSelection(OpMode opMode) {
        this.opMode = opMode;
        prompter = new Prompter(opMode);
        prompter.prompt("alliance", new AlliancePrompt());
    }

    public Alliance update() {
        if (prompter.isCompleted()) opMode.telemetry.addData("Alliance", getAlliance());
        prompter.run();
        return getAlliance();
    }

    public Alliance getAlliance() {
        return Robot.alliance;
    }

    private static class AlliancePrompt extends Prompt<Alliance> {

        @Override
        public Alliance process() {
            if (justPressed(Button.DPAD_UP) || justPressed(Button.DPAD_DOWN)) {
                Robot.alliance = Robot.alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
            }
            addLine("=== Select Alliance ===");
            addLine(" - RED" + (Robot.alliance == Alliance.RED ? " <" : ""));
            addLine(" - BLUE" + (Robot.alliance == Alliance.BLUE ? " <" : ""));
            return justPressed(Button.CROSS) ? Robot.alliance : null;
        }
    }
}
