package org.firstinspires.ftc.teamcode.DiagnosticOps;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Sensing.Ultrasonic;

/**
 * Created by Admin on 12/17/2017.
 */
@TeleOp(name = "UltrasonicTest1", group = "Festus")
public class UltrasonicTest1 extends LinearOpMode {


    Ultrasonic ultra;
    int distance;

    @Override
    public void runOpMode() throws InterruptedException {
        ultra= new Ultrasonic(hardwareMap,this);
        ultra.start();

        telemetry.addData(">", "Robot Ready!");
        telemetry.update();

        waitForStart();

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            distance = ultra.getDistance();
            telemetry.addData("cm", distance);
            telemetry.update();
        }

        ultra.stop();
        stop();
        
    }

}
