package org.firstinspires.ftc.teamcode.DiagnosticOps;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Sensing.Ultrasonic;

/**
 * Created by Admin on 12/17/2017.
 */
@TeleOp(name = "UltrasonicTest1", group = "Festus")
public class UltrasonicTest1 extends OpMode {


    Ultrasonic ultra;
    int distance;

    @Override
    public void init() {
        ultra= new Ultrasonic(hardwareMap);
        ultra.start();
    }

    @Override
    public void loop(){
        ultra.getDistance();
        telemetry.addData("cm", distance);
        telemetry.update();
    }

    public void stop(){
        ultra.stop();
    }
}
