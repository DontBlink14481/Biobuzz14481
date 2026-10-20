package org.firstinspires.ftc.teamcode.robot;

import android.sax.StartElementListener;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public class Robot {
    DcMotorEx intake, shooter, shooter2, transfer;
    CRServo turret1, turret2, transferWheel;
    Servo hood, shift, blocker;
    List<LynxModule> hubs;
    Follower f;

    public Robot(HardwareMap hw, boolean auto){
        this.initHardware(hw);

    }

    public Robot(HardwareMap hw){
        this(hw, false);
    }

    public void stop(){
        CommandScheduler.getInstance().reset();
        for(LynxModule x : hubs){
            x.clearBulkCache();
        }
    }
    public void initHardware(HardwareMap hw){
        f = Constants.create(hw);

        intake = hw.get(DcMotorEx.class, HardwareConstants.intake);
        shooter = hw.get(DcMotorEx.class, HardwareConstants.flywheel1Name);
        shooter2 = hw.get(DcMotorEx.class, HardwareConstants.flywheel1Name);
        transfer = hw.get(DcMotorEx.class, HardwareConstants.transfer);

        turret1 = hw.get(CRServo.class, HardwareConstants.turret1);
        turret2 = hw.get(CRServo.class, HardwareConstants.turret2);
        transferWheel = hw.get(CRServo.class, HardwareConstants.transferWheel);

        hood = hw.get(Servo.class, HardwareConstants.hoodName);
        shift = hw.get(Servo.class, HardwareConstants.shiftName);
        blocker = hw.get(Servo.class, HardwareConstants.blocker);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hubs = hw.getAll(LynxModule.class);

        for(LynxModule x : hubs)x.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

}
