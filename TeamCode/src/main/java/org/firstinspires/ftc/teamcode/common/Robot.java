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
import org.firstinspires.ftc.teamcode.common.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

public class Robot {

    public Follower f;

    public Intake intake;
    public Transfer transfer;
    public Turret turret;
    public Shooter shooter;
    public Blocker blocker;

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
        transfer = new Transfer(map);
        turret = new Turret(map);
        shooter = new Shooter(map);
        blocker = new Blocker(map);

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
