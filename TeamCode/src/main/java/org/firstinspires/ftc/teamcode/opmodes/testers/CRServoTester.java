package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "CRServo Tester")
public class CRServoTester extends LinearOpMode {
    public static String name = "turret1";
    public static double power = 0.0;
    public static boolean reversed = false;

    public void runOpMode() {
        MyTelem.init(telemetry);
        CRServo servo = hardwareMap.get(CRServo.class, name);
        CRServo servo2 = hardwareMap.get(CRServo.class, "turret2");
        CRServo servo3 = hardwareMap.get(CRServo.class, "turret3");


//        serv.o


        waitForStart();
        while (opModeIsActive()) {
            servo.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
            servo2.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
            servo3.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);


            servo.setPower(power);
            servo2.setPower(power);
            servo3.setPower(power);

            MyTelem.update();
        }
    }
}
