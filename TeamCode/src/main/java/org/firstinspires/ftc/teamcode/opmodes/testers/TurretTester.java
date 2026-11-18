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

    public static double x = 0, y = 0, headingDeg = 0;
    public static boolean auto = true;

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Turret turret = new Turret(hardwareMap);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        RobotStates.turretState = Turret.TurretState.OFF;
        turret.periodic();

        waitForStart();

        while (opModeIsActive()) {
            Robot.pose = new Pose(x, y, Math.toRadians(headingDeg));
            Robot.robotZone.setPosition(Robot.pose.x(), Robot.pose.y());
            Robot.robotZone.setRotation(Robot.pose.heading());
            Robot.updateGoal();

            RobotStates.turretState = auto
                    ? Turret.TurretState.CONTINUOUS : Turret.TurretState.MANUAL;
            if (!auto) {
                turret.setManualPower(gamepad1.right_stick_x);
            }
            turret.periodic();

            MyTelem.addData("enabled", auto);
            MyTelem.addData("robot turret state", RobotStates.turretState);
            MyTelem.addData("turret manual power", turret.manualPower);
            MyTelem.addData("1 right x", gamepad1.right_stick_x);
            MyTelem.addData("turret servo 1 power", turret.getPower());
            MyTelem.addData("turret servo 2 power", turret.getServo2Power());
            MyTelem.addData("pose x in", Robot.pose.x());
            MyTelem.addData("pose y in", Robot.pose.y());
            MyTelem.addData("pose heading deg", Math.toDegrees(Robot.pose.heading()));
            MyTelem.addData("goal", Robot.goal);
            MyTelem.addData("goal pose", Robot.getGoalPose());
            MyTelem.addData("turret target deg", Math.toDegrees(turret.getTargetAngle()));
            MyTelem.addData("turret angle deg", turret.getAngle(AngleUnit.DEGREES));
            MyTelem.addData("turret error deg", AngleUnit.normalizeDegrees(
                    Math.toDegrees(turret.getTargetAngle()) - turret.getAngle(AngleUnit.DEGREES)));
            MyTelem.addData("turret encoder voltage", turret.getEncoderVoltage());
            MyTelem.addData("turret encoder 2 voltage", turret.getEncoder2Voltage());
            MyTelem.addData("turret encoder 2 angle deg", turret.getEncoder2Angle(AngleUnit.DEGREES));
            MyTelem.update();
            idle();
        }

    }
}
