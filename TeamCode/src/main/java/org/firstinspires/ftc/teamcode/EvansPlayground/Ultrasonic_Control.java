package org.firstinspires.ftc.teamcode.EvansPlayground;


import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by nonba on 12/26/2017.
 */

public class Ultrasonic_Control {

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;

    Telemetry telemetry;

    I2CXL ultrasonicFront;
    I2CXL ultrasonicBack;
    I2CXL ultrasonicRight;
    I2CXL ultrasonicLeft;


    LinearOpMode linearOpMode;

    private double kp;//Proportional
    private double ki;//Integral
    private double kd;//Derivative
    private double integralMin;// The min of the running integral
    private double integralMax;// The max of the running integral
    private double Threshold; //
    double previousError;
    double runningIntegral;// The running instance of the integral bounded by the min and max.

    public Ultrasonic_Control(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {

        ultrasonicFront = hardwareMap.get(I2CXL.class, "ultsonFront");
        ultrasonicBack = hardwareMap.get(I2CXL.class, "ultsonBack");
        ultrasonicLeft = hardwareMap.get(I2CXL.class, "ultsonLeft");
        ultrasonicRight = hardwareMap.get(I2CXL.class, "ultsonRight");

        ultrasonicFront.initialize();
        ultrasonicBack.initialize();
        ultrasonicLeft.initialize();
        ultrasonicRight.initialize();

    }

    public Ultrasonic_Control(double kp, double ki, double kd, double integralMin,
                       double integralMax, double Threshold) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integralMin = integralMin;
        this.integralMax = integralMax;

        this.previousError = 0;
        this.runningIntegral = 0;
    }

    public static double clipValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public void ultrasonicPID (double targetDistanceInches, double measuredDistance){

        double targetDistance = targetDistanceInches * 2.54;;
        double error = targetDistance - measuredDistance;

        while (error > Threshold){

            error = targetDistance - measuredDistance;

            runningIntegral = clipValue(runningIntegral + error, integralMin, integralMax);
            double d = (error - previousError);
            double output = kp * (error + (runningIntegral / ki) + (kd * d));

            telemetry.addData("Output", output);

        }


    }

}
