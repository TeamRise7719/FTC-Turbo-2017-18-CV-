package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by seancardosi on 1/3/17.
 */

public class SeansEncLibrary {
    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;

    BNO055IMU gyro;
    Orientation angle;

    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public SeansEncLibrary(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {

        linearOpMode = opMode;

        //*************************************************************************************************************
        //                                  SETUP GYRO SENSOR
        //*************************************************************************************************************

        gyro = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angle = gyro.getAngularOrientation();

        telemetry = tel;
        left_back_drive = hardwareMap.dcMotor.get("1");
        left_front_drive = hardwareMap.dcMotor.get("2");
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_back_drive = hardwareMap.dcMotor.get("3");
        right_front_drive = hardwareMap.dcMotor.get("4");
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorToEnc();
    }

    public void setMotorToEnc(){
        //Reset Encoders
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Driving To Position
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorToPower(){
        //Drive with just Power
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //This function takes a target for the gyro to reach
    //Example a target of 90 degrees the bot will turn to hit 90 degrees on the gyro value
    public void rotateToTarget(double degrees, double power){
        //Get Gyro Angle
        angle = gyro.getAngularOrientation();
        //Calculate Current Angle to Target
        double difference = degrees - angle.firstAngle;
        while ((difference < -180)&&(linearOpMode.opModeIsActive())) {
            difference += 360;
            linearOpMode.idle();
        }
        while ((difference > 180)&&(linearOpMode.opModeIsActive())) {
            difference -= 360;
            linearOpMode.idle();
        }
        //Send off to rotate Function
        rotate(difference, power);
    }

    //   ------------------------------------------------   //

    // Function to rotate the robot the given amount of degrees
    public void rotate(double degrees, double power) {
        setMotorToPower();
        angle = gyro.getAngularOrientation();
        double targetAngle = Math.abs((degrees + angle.firstAngle) % 360);
        double firstAngle = Math.abs(angle.firstAngle);
        double TOLERANCE = 3.5;

        while((Math.abs(targetAngle - Math.abs(angle.firstAngle)) > TOLERANCE)&&(linearOpMode.opModeIsActive())){
            angle = gyro.getAngularOrientation();
            if(degrees < 0){
                left_back_drive.setPower(-power);
                right_back_drive.setPower(power);
                left_front_drive.setPower(-power);
                right_front_drive.setPower(power);
            }
            if(degrees > 0){
                left_back_drive.setPower(power);
                right_back_drive.setPower(-power);
                left_front_drive.setPower(power);
                right_front_drive.setPower(-power);
            }
            telemetry.addData("Gyro Target Angle:", targetAngle);
            telemetry.addData("Gyro Start Angle:", firstAngle);
            telemetry.update();
            linearOpMode.idle();
        }
        stop_all_motors();
        setMotorToEnc();
    }

    //Stop All Motors
    public void stop_all_motors(){
        left_back_drive.setPower(0);
        right_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Give a target for encoders and power to use motors.
    public void run_encoders(int distance, double power){
        setMotorToEnc();
        left_back_drive.setTargetPosition(distance);
        right_back_drive.setTargetPosition(distance);
        left_front_drive.setTargetPosition(distance);
        right_front_drive.setTargetPosition(distance);

        if (distance < 0) {
            left_back_drive.setPower(-power);
            right_back_drive.setPower(-power);
            left_front_drive.setPower(-power);
            right_front_drive.setPower(-power);
            while (((left_back_drive.getCurrentPosition() > distance) || (right_back_drive.getCurrentPosition() > distance))&&(linearOpMode.opModeIsActive())) {
                linearOpMode.idle();
            }
        }
        else {
            left_back_drive.setPower(power);
            right_back_drive.setPower(power);
            left_front_drive.setPower(power);
            right_front_drive.setPower(power);
            while(((left_back_drive.getCurrentPosition() < distance || right_back_drive.getCurrentPosition() < distance))&&(linearOpMode.opModeIsActive())) {
            linearOpMode.idle();
            }
        }
        setMotorToPower();
        stop_all_motors();
    }

    //Evan - Zach can you look this over when you get the chance I'm not sure I did it right (I never am)
    //This is a function to take an input in inches and convert it to the appropriate encoder values
    //based off of data collected from our trials with the robot.
    public void run_encoders_inch(double distance, double power){
        setMotorToEnc();
        distance = distance / 0.017;
        int enc_distance = (int) Math.round(distance);

        left_back_drive.setTargetPosition(enc_distance);
        right_back_drive.setTargetPosition(enc_distance);
        left_front_drive.setTargetPosition(enc_distance);
        right_front_drive.setTargetPosition(enc_distance);



        if (distance < 0) {
            left_back_drive.setPower(-power);
            right_back_drive.setPower(-power);
            left_front_drive.setPower(-power);
            right_front_drive.setPower(-power);
            while ((left_back_drive.getCurrentPosition() < distance || right_back_drive.getCurrentPosition() < distance)&&(linearOpMode.opModeIsActive())) {
                linearOpMode.idle();
        }
        }
        else {
            left_back_drive.setPower(power);
            right_back_drive.setPower(power);
            left_front_drive.setPower(power);
            right_front_drive.setPower(power);
            while ((left_back_drive.getCurrentPosition() > distance || right_back_drive.getCurrentPosition() > distance)&&(linearOpMode.opModeIsActive())){
                linearOpMode.idle();
            }
        }
        stop_all_motors();
    }
}