package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Driving.PID_Library;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;
import org.firstinspires.ftc.teamcode.subsystems.Sensing.RobotVision;

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
