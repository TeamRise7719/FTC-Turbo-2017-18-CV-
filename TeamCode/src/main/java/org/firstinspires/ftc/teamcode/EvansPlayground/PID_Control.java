package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Evan on 12/4/2017.
 */

//Behold the beauty of Wikipedia:
//https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
//
//Standard form of the PID formula:
//kp * (e + (integral(e) / ti) + (td * derivative(e)))

public class PID_Control {

    Math math;

    DcMotor lb;
    DcMotor lf;
    DcMotor rb;
    DcMotor rf;

    BNO055IMU gyro;
    Orientation gyro_angle;

    Telemetry telemetry;
    LinearOpMode linearOpMode;

    ElapsedTime eTime;

    public PID_Control(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {

        lb = hardwareMap.dcMotor.get("1");
        lf = hardwareMap.dcMotor.get("2");
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb = hardwareMap.dcMotor.get("3");
        rf = hardwareMap.dcMotor.get("4");
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        gyro_angle = gyro.getAngularOrientation();

        telemetry = tel;
        linearOpMode = opMode;

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    private double kp;//Proportional
    private double ti;//Integral
    private double td;//Derivative
    private double integralMin;// The min of the running integral
    private double integralMax;// The max of the running integral
    private double encThreshold; //
    double previousError;
    double runningIntegral;// The running instance of the integral bounded by the min and max.


    // TicksPerMotorRev --- This is the ticks on the encoder per revolution of the motor shaft
    private static final double ticksPerMotorRev = 1120;
    // driveTrainReduction --- The gear reduction ratio from the motor to the whee. This is the number of rotations of the wheel per rotation of the motor shaft
    private static final double drivetrainReduction = 0.66667;
    //wheelDiamInches --- This is the diameter of the wheel in inches. Make sure to convert from metric!
    private static final double wheelDiamInches = 3.937;
    //pi --- self explanatory
    private static final double pi = 3.14159;
    //ticksPerInch --- This is where the last three parameters are implemented to determine the ticks on the encoder per inch driven
    private static final double ticksPerInch = (ticksPerMotorRev * drivetrainReduction) / (wheelDiamInches * pi);


    public PID_Control(double kp, double ti, double td, double integralMin,
                       double integralMax, double encThreshold) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.integralMin = integralMin;
        this.integralMax = integralMax;

        this.previousError = 0;
        this.runningIntegral = 0;
    }

    public static double clipValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public double update(double desiredValue, double actualValue, double dt) {

        double e = desiredValue - actualValue; //Calc error

        runningIntegral = clipValue(runningIntegral + e * dt, integralMin, integralMax);
        double d = (e - previousError) / dt;
        double output = kp * (e + (runningIntegral / ti) + (td * d));

        previousError = e;
        return output;
    }


    public void PID_Loop(double targetDistance, double targetAngle) {

        int distance = (int) (targetDistance * ticksPerInch);
        int realLeftDistance;
        int realRightDistance;
        double leftSpeed;
        double rightSpeed;
        double currentEnc;
        int error;
        int prev_error;


        if (linearOpMode.opModeIsActive()) {

            realLeftDistance = lb.getCurrentPosition() + distance;
            realRightDistance = rb.getCurrentPosition() + distance;


            lb.setTargetPosition(realLeftDistance);
            lf.setTargetPosition(realLeftDistance);
            rb.setTargetPosition(realRightDistance);
            rf.setTargetPosition(realRightDistance);

            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            while (math.abs(error) > encThreshold){
//
//                error = desiredValue - actualValue; //Calc error
//
//                runningIntegral = clipValue(runningIntegral + e * dt, integralMin, integralMax);
//                double d = (e - previousError) / dt;
//                double output = kp * (e + (runningIntegral / ti) + (td * d));
//
//                previousError = e;
//                return output;
//        }

        }


    }
}