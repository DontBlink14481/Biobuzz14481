package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.utils.MyTelem;
import org.firstinspires.ftc.teamcode.utils.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.utils.constants.TurretConstants;

@Config
@TeleOp(name = "Turret Servo Tester")
public class TurretServoTester extends LinearOpMode {
    public static String name1 = "turret1";
    public static double position = 0.5;
    public void runOpMode() {
        MyTelem.init(telemetry);
        Servo servo = hardwareMap.get(Servo.class, HardwareConstants.turret1);
        Servo servo2 = hardwareMap.get(Servo.class, HardwareConstants.turret2);
        Servo servo3 = hardwareMap.get(Servo.class, HardwareConstants.turret3);
        servo3.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(position);
            servo2.setPosition(position);
            servo3.setPosition(position);
            double expectedAngle = position - 0.5;
            expectedAngle /= TurretConstants.posPerDeg;

            MyTelem.addData("expected Angle", expectedAngle);
            MyTelem.update();
        }
    }
}
