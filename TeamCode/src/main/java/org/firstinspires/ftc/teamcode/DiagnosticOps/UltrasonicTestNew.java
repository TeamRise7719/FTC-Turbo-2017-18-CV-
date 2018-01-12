package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Driving.PID_Library;

/**
 * Created by Evan McLoughlin on 12/18/2017.
 */

@Autonomous
public class UltrasonicTestNew extends LinearOpMode {

    PID_Library enc;

    @Override
    public void runOpMode() throws InterruptedException {

        enc = new PID_Library(hardwareMap, telemetry,this);
        enc.init();

        telemetry.addData(">", "Robot Ready!");
        telemetry.update();
        waitForStart();

        enc.UltrasonicGyroDrive( 20, 0,false, 0.25, true, 10);


    }
}
