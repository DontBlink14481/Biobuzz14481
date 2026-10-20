package org.firstinspires.ftc.teamcode.opmodes.testers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "Example TeleOp")
public class MechDrive extends OpMode {
    private Follower follower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }
    @Override
    public void loop() {
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();
    }
}