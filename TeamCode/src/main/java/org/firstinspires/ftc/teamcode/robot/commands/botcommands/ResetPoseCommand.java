package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.constants.RobotConstants;

public class ResetPoseCommand extends InstantCommand {
    public ResetPoseCommand(Robot robot){
        super(() -> robot.follower.setPose(RobotConstants.resetPose.pose()));
    }
}
