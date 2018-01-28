package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Driving.SeansEncLibrary;

/**
 * Created by seancardosi on 10/25/17.
 */
@TeleOp(name = "TurnPID Test", group = "Festus")
public class TurnPID extends LinearOpMode {

    SeansEncLibrary enc;

    public void runOpMode() throws InterruptedException {


        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        waitForStart();

        while (opModeIsActive()) {
            enc.gyroHold(enc.TURN_SPEED, 90, 10);
        }
    }
}
