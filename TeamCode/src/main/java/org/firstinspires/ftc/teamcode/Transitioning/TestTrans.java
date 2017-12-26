package org.firstinspires.ftc.teamcode.Transitioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Transitioning.AutoTransitioner;

/**
 * Created by seancardosi on 12/18/17.
 */
@Autonomous(name = "TestTransitioner")
public class TestTrans extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing Here", true);
        telemetry.update();


        waitForStart();

        AutoTransitioner.transitionOnStop(this, "EvanTeleOp");

    }
}