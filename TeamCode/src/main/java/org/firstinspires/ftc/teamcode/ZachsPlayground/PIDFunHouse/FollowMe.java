package org.firstinspires.ftc.teamcode.ZachsPlayground.PIDFunHouse;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.PID_Library;

@TeleOp(name="FollowMe", group ="Concept")
@Disabled
public class FollowMe extends LinearOpMode {

    FollowMeDrive enc;

    @Override
    public void runOpMode() {
        enc = new FollowMeDrive(hardwareMap, telemetry,this);
        waitForStart();

        while (opModeIsActive()) {
            enc.gyroDrive();
            break;
        }

    }
}
