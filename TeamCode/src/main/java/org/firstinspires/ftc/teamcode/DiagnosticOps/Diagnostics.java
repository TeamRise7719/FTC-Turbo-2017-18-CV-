package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by seancardosi on 6/17/17.
 */
@TeleOp(name = "TouchColor", group = "Rise Robot")
@Disabled
public class Diagnostics extends OpMode {

    ColorSensor color;
    TouchSensor touch;


    public void init() {

        color = hardwareMap.colorSensor.get("color");
        touch = hardwareMap.touchSensor.get("touch");

    }

    public void loop() {


        telemetry.addData("red", color.red());
        telemetry.addData("blue", color.blue());
        telemetry.addData("green", color.green());
        telemetry.addData("alpha", color.alpha());
        telemetry.addData("hue", color.argb());
        telemetry.addData("TouchValue", touch.getValue());


    }
}
