package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by seancardosi on 12/26/17.
 */
@TeleOp(name = "I2C_Bus_Test", group = "ultra")
public class I2C_Bus_Test extends OpMode {

    I2CXL leftUltra;
    I2CXL rightUltra;
    I2CXL frontUltra;
    I2CXL backUltra;

    public void init() {

        leftUltra = hardwareMap.get(I2CXL.class, "leftUltra");
        leftUltra.initialize();
        rightUltra = hardwareMap.get(I2CXL.class, "rightUltra");
        rightUltra.initialize();
        frontUltra = hardwareMap.get(I2CXL.class, "frontUltra");
        frontUltra.initialize();
        backUltra = hardwareMap.get(I2CXL.class, "backUltra");
        backUltra.initialize();
    }

    public void loop() {
        telemetry.addData("leftUltra", leftUltra);
        telemetry.addData("rightUltra", rightUltra);
        telemetry.addData("frontUltra", frontUltra);
        telemetry.addData("backUltra", backUltra);
    }
}
