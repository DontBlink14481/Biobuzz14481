package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public class Robot {

    Follower f;

    Intake intake;
    DcMotorEx transfer, shooter, shooter2;
    CRServo turret1, turret2, dropdown;
    Servo hood, shift, blocker, pinch;
    List<LynxModule> hubs;

    public Robot(HardwareMap map) {
        this(map, false);
    }

    public Robot(HardwareMap map, boolean auto) {
        this.initHardware(map);
    }

    public void initHardware(HardwareMap map) {
        f = Constants.create(map);

        intake = new Intake(map);

        transfer = map.get(DcMotorEx.class, HardwareConstants.transfer);
        shooter = map.get(DcMotorEx.class, HardwareConstants.shooter1);
        shooter2 = map.get(DcMotorEx.class, HardwareConstants.shooter1);

        turret1 = map.get(CRServo.class, HardwareConstants.turret1);
        turret2 = map.get(CRServo.class, HardwareConstants.turret2);
        dropdown = map.get(CRServo.class, HardwareConstants.dropdown);

        hood = map.get(Servo.class, HardwareConstants.hood);
        shift = map.get(Servo.class, HardwareConstants.shift);
        blocker = map.get(Servo.class, HardwareConstants.blocker);
        pinch = map.get(Servo.class, HardwareConstants.pinch);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hubs = map.getAll(LynxModule.class);

        for (LynxModule x : hubs) x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void stop() {
        CommandScheduler.getInstance().reset();
        for (LynxModule x : hubs) {
            x.clearBulkCache();
        }
    }

}
