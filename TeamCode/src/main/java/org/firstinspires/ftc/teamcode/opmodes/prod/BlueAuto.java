package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto", group = "Prod")
@Config
public class BlueAuto extends RedAuto {
    public BlueAuto() { super("BLUE"); }
}