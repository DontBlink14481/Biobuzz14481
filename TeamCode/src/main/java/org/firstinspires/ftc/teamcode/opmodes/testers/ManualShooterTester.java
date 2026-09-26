package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.MyTelem;

@Config
@TeleOp(name = "Manual Shooter Tester")
public class ManualShooterTester extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    Servo hood;
    Servo shift;
    CRServo dropdown;

    public static MotorDirections direction = MotorDirections.SAME;
    public static ManualMode manualMode = ManualMode.OFF;
    public static ShiftMode shiftMode = ShiftMode.NECTAR;
    public static double targetRPM = 3200;
    public static double power = 0.0;
    public static double hoodPos = 0.75;

    public void runOpMode(){
        MyTelem.init(telemetry);

        motor1 = hardwareMap.get(DcMotorEx.class, HardwareConstants.flywheel1Name);
        motor2 = hardwareMap.get(DcMotorEx.class, HardwareConstants.flywheel2Name);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, HardwareConstants.hoodName);
//        shift = hardwareMap.get(Servo.class, HardwareConstants.shiftName);

//        dropdown = hardwareMap.get(CRServo.class, HardwareConstants.dropdownName);

        switch(direction) {
            case SAME:
                motor1.setDirection(DcMotorEx.Direction.REVERSE);
                motor2.setDirection(DcMotorEx.Direction.REVERSE);
                break;
            case OPPOSITE:
                motor1.setDirection(DcMotorEx.Direction.REVERSE);
                motor2.setDirection(DcMotorEx.Direction.FORWARD);
                break;
        }

        waitForStart();

        while(opModeIsActive()) {
            if (manualMode == ManualMode.OFF) {
                power = (Math.abs(targetRPM) - Math.abs(getRPM()) > 0) ? 1 : 0;
            }


        motor1.setPower(power);
        motor2.setPower(power);

        hood.setPosition(Math.min(ShooterConstants.hoodMax, Math.max(ShooterConstants.hoodMin, hoodPos)));

//            switch (shiftMode) {
//                case POLLEN:
//                    shift.setPosition(ShooterConstants.pollenPos);
//                    break;
//                case NECTAR:
//                    shift.setPosition(ShooterConstants.nectarPos);
//                    break;
//            }

        MyTelem.addData("power", motor1.getPower());
        MyTelem.addData("target rpm", targetRPM);
        MyTelem.addData("rpm", getRPM());
        MyTelem.addData("hood pos", hood.getPosition());
        MyTelem.addData("hood insert pos", hoodPos);
        MyTelem.update();
        }
    }


    public double getRPM() {
        return (motor1.getVelocity() * 60) / 28;
    }

    public enum MotorDirections{
        OPPOSITE, SAME
    }
    public enum ManualMode{
        ON, OFF
    }

    public enum ShiftMode {
        POLLEN,
        NECTAR
    }
}
