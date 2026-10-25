package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.common.constants.HardwareConstants;

public class Turret implements Subsystem {
    CRServo turret1;
    CRServo turret2;

    public Turret(HardwareMap map) {
        turret1 = map.get(CRServo.class, HardwareConstants.turret1);
        turret2 = map.get(CRServo.class, HardwareConstants.turret2);
    }

    @Override
    public void periodic() {

    }

    public void setPower(double power) {
        turret1.setPower(power);
        turret2.setPower(power);
    }

}
