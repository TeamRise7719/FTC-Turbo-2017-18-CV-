package org.firstinspires.ftc.teamcode.ZachsPlayground.PIDFunHouse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class FollowMeDrive {

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;

    FollowMeVision vMod;

    Telemetry telemetry;
    LinearOpMode linearOpMode;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public  final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.

    private static final double     P_DRIVE_COEFF           = 0.16;     // Larger is more responsive, but also less stable
    private static boolean steeringToggle = true;

    public FollowMeDrive(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {

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

        telemetry = tel;
        linearOpMode = opMode;

        vMod = new FollowMeVision(hardwareMap, telemetry);
        vMod.init();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void disableSteering(){steeringToggle = false;}
    public void enableSteering(){steeringToggle = true;}

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

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     */
    public void gyroDrive () {
        double  max;
        double  errorSpeed;
        double  errorAngle;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double speed;
        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive()) {

                vMod.getVuMark();

                if (vMod.vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    // adjust relative speed based on heading error.
                    errorAngle = getErrorAngle(0);
                    steer = getSteer(errorAngle, P_DRIVE_COEFF);
                    errorSpeed = getErrorDistance(0);

                    speed = getSpeed(errorSpeed, P_DRIVE_COEFF);


                    // if driving in reverse, the motor correction also needs to be reversed
                    if (vMod.tX > 0) {
                        steer *= -1.0;
                    }


                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;


                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    left_back_drive.setPower(leftSpeed);
                    left_front_drive.setPower(leftSpeed);
                    right_back_drive.setPower(rightSpeed);
                    right_front_drive.setPower(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", errorAngle, steer);
                    telemetry.addData("Err", "%5.1f", errorSpeed);
                    telemetry.addData("X/Y/Z", "%5.1f/%5.1f/%5.1f", vMod.tX, vMod.tY, vMod.tZ);
                    telemetry.addData("Angles", "%5.1f/%5.1f/%5.1f", vMod.rX, vMod.rY, vMod.rZ);
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }
                else{
                    telemetry.addData("Status", "VuMark Not Found");
                    telemetry.update();
                    // Stop all motion;
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                }
            }

            // Stop all motion;
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);
        }
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getErrorAngle(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - vMod.rY/10;

        while (robotError > 15)  robotError -= 15;
        while (robotError <= -15) robotError += 15;
        return robotError;
    }

    public double getErrorDistance(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - vMod.rX/80;

        while (robotError > 70)  robotError -= 70;
        while (robotError <= -70) robotError += 70;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getSpeed(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}