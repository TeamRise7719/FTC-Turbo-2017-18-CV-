package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Evan McLoughlin on 12/18/2017.
 */

public class UltrasonicTestNew extends OpMode {

    I2CXL ultrasonic;

    @Override
    public void init() {

        ultrasonic = hardwareMap.get(I2CXL.class, "ultrasonic");

    }

    @Override
    public void loop() {

        telemetry.addData("Distance", ultrasonic.getDistance());

    }

}
