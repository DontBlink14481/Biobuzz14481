package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.RobotStates;

@Config
@TeleOp(name = "TurretTester")
public class TurretTester extends LinearOpMode {
    public static boolean auto = true;
    public static double position = 0.5;

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Robot robot = new Robot(hardwareMap);
        RobotStates.turretState = Turret.TurretState.OFF;
        robot.turret.periodic();
        robot.follower.setPose(new Pose(72, 72, 0));
        waitForStart();

        while (opModeIsActive()) {
//            Robot.pose = new Pose(x, y, Math.toRadians(headingDeg));
//            Robot.robotZone.setPosition(Robot.pose.x(), Robot.pose.y());
//            Robot.robotZone.setRotation(Robot.pose.heading());
//            Robot.updateGoal();

            RobotStates.turretState = auto
                    ? Turret.TurretState.SET : Turret.TurretState.MANUAL;
            if (!auto) {
                robot.turret.setManualPosition(position);
            }

            MyTelem.addData("enabled", auto);
            MyTelem.addData("robot turret state", RobotStates.turretState);
            MyTelem.addData("turret servo 1 position", robot.turret.getPosition());
            MyTelem.addData("turret servo 2 position", robot.turret.getServo2Position());
            MyTelem.addData("turret servo 3 position", robot.turret.getServo3Position());
            MyTelem.addData("pose x in", Robot.pose.x());
            MyTelem.addData("pose y in", Robot.pose.y());
            MyTelem.addData("pose heading deg", Math.toDegrees(Robot.pose.heading()));
            MyTelem.addData("goal", Robot.goal);
            MyTelem.addData("goal pose", Robot.getGoalPose());
            MyTelem.addData("turret target deg", Math.toDegrees(robot.turret.getTargetAngle()));
            MyTelem.addData("turret angle deg", robot.turret.getAngle(AngleUnit.DEGREES));
            MyTelem.addData("turret error deg", AngleUnit.normalizeDegrees(
                    Math.toDegrees(robot.turret.getTargetAngle()) - robot.turret.getAngle(AngleUnit.DEGREES)));
            MyTelem.addData("turret encoder voltage", robot.turret.getEncoderVoltage());
            MyTelem.addData("turret encoder 2 voltage", robot.turret.getEncoder2Voltage());
            MyTelem.addData("turret encoder 2 angle deg", robot.turret.getEncoder2Angle(AngleUnit.DEGREES));
            robot.update();
            idle();
        }

    }
}
