package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class OctoBeamBreak extends SubsystemBase {

    private OctoQuad octo;
    private int port;
    private ElapsedTime timer = new ElapsedTime();

    private int count;
    private int start;
    private boolean last;
    private boolean blocked;

    public static double DEBOUNCE = 25;

    public OctoBeamBreak(HardwareMap hardwareMap, int port) {
        octo = hardwareMap.get(OctoQuad.class, "octoquad");
        this.port = port;
        zero(false);
    }

    public void zero(boolean blockedNow) {
        count = octo.readSinglePosition_Caching(port);
        if (blockedNow) {
            start = count - 1;
        } else {
            start = count;
        }
        last = blockedNow;
        blocked = blockedNow;
        timer.reset();
    }

    @Override
    public void periodic() {
        count = octo.readSinglePosition_Caching(port);

        //odd = something in front of it
        boolean now = (count - start) % 2 != 0;

        if (now != last) {
            last = now;
            timer.reset();
        }
        if (timer.milliseconds() > DEBOUNCE) {
            blocked = now;
        }
    }

    public boolean isBlocked() {
        return blocked;
    }

    public int getCount() {
        return count;
    }
}