package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/**
 * Created by seancardosi on 7/8/17.
 */
@TeleOp(name = "IRTest", group = "Festus")
@Disabled
public class IRTest extends OpMode {


    IrSeekerSensor ir;

    public void init() {

        ir = hardwareMap.irSeekerSensor.get("ir");

    }

    public void loop() {

        double angle = ir.getAngle();
        double strength = ir.getStrength();
        boolean detected = ir.signalDetected();



        telemetry.addData("angle", angle);
        telemetry.addData("detected", detected);
        telemetry.addData("strength", strength);

    }
}
