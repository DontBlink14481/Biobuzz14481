package org.firstinspires.ftc.teamcode.opmodes.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.AllianceSelection;

@TeleOp(name = "Choose Alliance", group = "Setup")
public class ChooseAlliance extends LinearOpMode {
    @Override
    public void runOpMode() {
        AllianceSelection allianceSelection = new AllianceSelection(this);
        while (!isStopRequested()) {
            allianceSelection.update();
            idle();
        }
    }
}
