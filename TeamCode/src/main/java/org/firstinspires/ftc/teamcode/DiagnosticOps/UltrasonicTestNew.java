package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Evan McLoughlin on 12/18/2017.
 */
@TeleOp
public class UltrasonicTestNew extends OpMode {

    I2CXL ultrasonic;


    @Override
    public void init() {

        ultrasonic = hardwareMap.get(I2CXL.class, "ultson");
        ultrasonic.initialize();


    }

    @Override
    public void loop() {

        telemetry.addData("Distance", ultrasonic.sampleDistance());

    }

}
