package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Sensing.RobotVision;

/**
 * Created by seancardosi on 6/14/17.
 */
@TeleOp(name = "VuforiaTester", group = "Festus")
public class VuforiaTest extends OpMode {

    //naming the components that will be used in the program
    RobotVision vMod;

    public void init() {
        vMod = new RobotVision(hardwareMap,telemetry);
        vMod.init();
    }

    public void loop() {
        vMod.getVuMark();
        telemetry.addData("VuMark", vMod.vuMark);
        telemetry.update();
    }
}
