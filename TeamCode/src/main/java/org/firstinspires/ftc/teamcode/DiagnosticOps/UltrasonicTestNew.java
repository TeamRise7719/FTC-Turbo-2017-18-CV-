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

    FestusDrivetrain robot;



    DcMotor liftMotor;
    ColorSensor color;


    RobotVision vMod;
    ServoManagementV2 srvo;
    PID_Library enc;

    ElapsedTime etime = new ElapsedTime();

    int position = 0;

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        enc = new PID_Library(hardwareMap, telemetry,this);

        waitForStart();

        enc.UltrasonicGyroDrive( 20, 0,false, 0.25, true);


    }
}
