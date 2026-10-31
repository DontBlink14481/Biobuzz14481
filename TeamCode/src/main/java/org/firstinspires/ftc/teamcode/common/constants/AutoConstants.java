package org.firstinspires.ftc.teamcode.common.constants;

import com.acmerobotics.dashboard.config.Config;

// Numbers the auto uses to shoot and collect. All of them are guesses until someone tunes them:
// find the RPM and hood with the Clean Intake Shooter Tester, time a real volley, then copy them here.
@Config
public class AutoConstants {

    public static double shooterRPM = 3000;          // PLACEHOLDER
    public static double rpmWindow = 100;            // start feeding once the RPM is this close to the target
    public static double spinUpTimeoutS = 1.0;       // feed anyway after this long, so a slow flywheel can't stall the auto

    public static double hoodPos = 0.6;              // the tester's default
    public static double nectarHoodOffset = 0.1;     // the tester's default

    // Keep this equal to the 0.3 in autos/robot.json's score estimate, so Zenith's times match the robot.
    public static double feedSecondsPerPiece = 0.3;  // PLACEHOLDER

    // CHECK: does the transfer need to run while intaking to pull pieces in? The blocker stays closed either way.
    public static boolean transferWhileIntaking = true;
}
