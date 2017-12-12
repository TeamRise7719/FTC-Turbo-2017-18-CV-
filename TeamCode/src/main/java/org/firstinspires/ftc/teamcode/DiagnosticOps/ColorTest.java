package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by seancardosi on 6/14/17.
 */
@TeleOp(name = "ColorTest", group = "Festus")
@Disabled
public class ColorTest extends OpMode {
    //Import 2017 Drivetrain Subsystems
    ColorSensor color_sensor;




    public void init() {
        color_sensor = hardwareMap.colorSensor.get("color");

    }

    public void loop() {

        telemetry.addData("R", color_sensor.red());
        telemetry.addData("G", color_sensor.green());
        telemetry.addData("B", color_sensor.blue());
        telemetry.update();
    }
}
