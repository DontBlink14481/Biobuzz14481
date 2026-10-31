package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// B for red, X for blue run that for alliance
@TeleOp(name = "Choose Alliance", group = "Setup")
public class ChooseAlliance extends LinearOpMode {

    public static volatile String alliance = "RED";

    @Override
    public void runOpMode() {
        while (!isStopRequested()) {
            if (gamepad1.b) alliance = "RED";
            if (gamepad1.x) alliance = "BLUE";

            telemetry.addData("Alliance", alliance);
            telemetry.addLine("B for RED, X for BLUE, then stop this OpMode.");
            telemetry.update();
            idle();
        }
    }
}
