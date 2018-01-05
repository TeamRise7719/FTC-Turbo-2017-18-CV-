package org.firstinspires.ftc.teamcode.subsystems.Driving;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PID_Library {

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;

    BNO055IMU gyro;
    Orientation gyro_angle;

    Telemetry telemetry;
    LinearOpMode linearOpMode;
    ElapsedTime etime;

    I2CXL ultrasonicFront;
    I2CXL ultrasonicBack;

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 0.6666;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public  final double     DRIVE_SPEED             = 0.8;     // Nominal speed
    public  final double     DRIVE_SPEED_SLOW             = 0.4;     // Slower speed for better accuracy.

    public  final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     ENCODER_THRESHOLD       = 10;      // As tight as we can make it with an integer gyro

    private static final double     P_TURN_COEFF            = 0.8;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.16;     // Larger is more responsive, but also less stable

    public PID_Library(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {

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

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        gyro_angle = gyro.getAngularOrientation();

        telemetry = tel;
        linearOpMode = opMode;
        ElapsedTime etime = new ElapsedTime();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ultrasonicFront = hardwareMap.get(I2CXL.class, "ultsonFront");
        ultrasonicBack = hardwareMap.get(I2CXL.class, "ultsonBack");

        ultrasonicFront.initialize();
        ultrasonicBack.initialize();

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

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle,
                            boolean steeringToggle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
            newRightTarget = right_back_drive.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            left_back_drive.setTargetPosition(newLeftTarget);
            left_front_drive.setTargetPosition(newLeftTarget);
            right_back_drive.setTargetPosition(newRightTarget);
            right_front_drive.setTargetPosition(newRightTarget);

            left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left_back_drive.setPower(speed);
            left_front_drive.setPower(speed);
            right_back_drive.setPower(speed);
            right_front_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive() && (Math.abs(newLeftTarget-left_back_drive.getCurrentPosition())>ENCODER_THRESHOLD || Math.abs(newRightTarget-right_back_drive.getCurrentPosition())>ENCODER_THRESHOLD) ) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance > 0) {
                    steer *= -1.0;
                }

                if(steeringToggle) {
                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;
                }
                else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left_back_drive.setPower(-leftSpeed);
                left_front_drive.setPower(-leftSpeed);
                right_back_drive.setPower(-rightSpeed);
                right_front_drive.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      left_back_drive.getCurrentPosition(),
                        right_back_drive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  -leftSpeed, -rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void gyroDriveTime ( double speed,
                                double time,
                            double angle,
                            boolean steeringToggle) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Set Target and Turn On RUN_TO_POSITION
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left_back_drive.setPower(speed);
            left_front_drive.setPower(-speed);
            right_back_drive.setPower(-speed);
            right_front_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            etime.reset();
            while ((etime.time() < time)&&(linearOpMode.opModeIsActive())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (speed > 0) {
                    steer *= -1.0;
                }

                if(steeringToggle) {
                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;
                }
                else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left_back_drive.setPower(-leftSpeed);
                left_front_drive.setPower(-leftSpeed);
                right_back_drive.setPower(-rightSpeed);
                right_front_drive.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  -leftSpeed, -rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroStrafeTime ( double speed,
                            double time,
                            double angle,
                                 boolean steeringToggle) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left_back_drive.setPower(speed);
            left_front_drive.setPower(-speed);
            right_back_drive.setPower(-speed);
            right_front_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            etime.reset();
            while ((etime.time() < time)&&(linearOpMode.opModeIsActive())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (speed > 0) {
                    steer *= -1.0;
                }

                if(steeringToggle) {
                    rightSpeed = speed - steer;
                    leftSpeed = speed + steer;
                }
                else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left_back_drive.setPower(-leftSpeed);
                left_front_drive.setPower(leftSpeed);
                right_back_drive.setPower(-rightSpeed);
                right_front_drive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  -rightSpeed, -leftSpeed);
                telemetry.update();
            }

            // Stop all motion;
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);

            // Run Mode RUN_USING_ENCODER
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroStrafeDistance ( double speed,
                             double distance,
                             double angle,
                                     boolean steeringToggle) {


        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
            newRightTarget = (right_back_drive.getCurrentPosition() + moveCounts)*-1;


            // Set Target and Turn On RUN_TO_POSITION
            left_back_drive.setTargetPosition(newLeftTarget);
            left_front_drive.setTargetPosition(-newLeftTarget);
            right_back_drive.setTargetPosition(newRightTarget);
            right_front_drive.setTargetPosition(-newRightTarget);

            left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left_back_drive.setPower(speed);
            left_front_drive.setPower(-speed);
            right_back_drive.setPower(-speed);
            right_front_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive() && (Math.abs(newLeftTarget-left_back_drive.getCurrentPosition())>ENCODER_THRESHOLD || Math.abs((newRightTarget)-right_back_drive.getCurrentPosition())>ENCODER_THRESHOLD) ) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance > 0) {
                    steer *= -1.0;
                }

                if(steeringToggle) {
                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;
                }
                else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left_back_drive.setPower(-rightSpeed);
                left_front_drive.setPower(rightSpeed);
                right_back_drive.setPower(-leftSpeed);
                right_front_drive.setPower(leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      left_back_drive.getCurrentPosition(),
                        right_back_drive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  -rightSpeed, -leftSpeed);
                telemetry.update();
            }

            // Stop all motion;
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {
         boolean doneTurning = false;
        // keep looping while we are still active, and not on heading.
        while (linearOpMode.opModeIsActive() && !doneTurning) {
            // Update telemetry & Allow time for other processes to run.
             doneTurning = onHeading(speed, angle, P_TURN_COEFF);
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (linearOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        left_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_back_drive.setPower(0);
        right_front_drive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = (speed * steer);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        left_back_drive.setPower(-leftSpeed);
        left_front_drive.setPower(-leftSpeed);
        right_back_drive.setPower(-rightSpeed);
        right_front_drive.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Angle", gyro_angle.firstAngle);
        telemetry.addData("Speed.", "%5.2f:%5.2f", -leftSpeed, -rightSpeed);
        telemetry.update();

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        gyro_angle = gyro.getAngularOrientation();
        robotError = targetAngle - gyro_angle.firstAngle;

        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
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

    public void UltrasonicGyroDrive(double speed,
                                             double distance,
                                             double angle,
                                             boolean steeringToggle,
                                             double UltraTolerance,
                                             boolean isBack) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;



        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            double ultrasonicValue;

            //Determine whether to use back or front and then convert to freedom units

            if (isBack) {

                //use the back sensor
                ultrasonicValue = (ultrasonicBack.sampleDistance() / 2.54);

            } else {

                //use the front sensor
                ultrasonicValue = (ultrasonicFront.sampleDistance() / 2.54);

            }



            //take the first calculation of the distance error
            double ultraError = distance - ultrasonicValue;

            while (linearOpMode.opModeIsActive() && java.lang.Math.abs(ultraError) > UltraTolerance) {




                moveCounts = (int)(ultraError * COUNTS_PER_INCH);



                // Determine new target position, and pass to motor controller

                newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
                newRightTarget = right_back_drive.getCurrentPosition() + moveCounts;


                // Set Target and Turn On RUN_TO_POSITION
                left_back_drive.setTargetPosition(newLeftTarget);
                left_front_drive.setTargetPosition(newLeftTarget);
                right_back_drive.setTargetPosition(newRightTarget);
                right_front_drive.setTargetPosition(newRightTarget);

                left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                left_back_drive.setPower(speed);
                left_front_drive.setPower(speed);
                right_back_drive.setPower(speed);
                right_front_drive.setPower(speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (linearOpMode.opModeIsActive() && (Math.abs(newLeftTarget - left_back_drive.getCurrentPosition()) > ENCODER_THRESHOLD || Math.abs(newRightTarget - right_back_drive.getCurrentPosition()) > ENCODER_THRESHOLD)) {
                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance > 0) {
                        steer *= -1.0;
                    }

                    if (steeringToggle) {
                        leftSpeed = speed - steer;
                        rightSpeed = speed + steer;
                    } else {
                        leftSpeed = speed;
                        rightSpeed = speed;
                    }

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    left_back_drive.setPower(-leftSpeed);
                    left_front_drive.setPower(-leftSpeed);
                    right_back_drive.setPower(-rightSpeed);
                    right_front_drive.setPower(-rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Actual", "%7d:%7d", left_back_drive.getCurrentPosition(),
                            right_back_drive.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", -leftSpeed, -rightSpeed);
                    telemetry.update();
                }

                // Stop all motion;
                left_back_drive.setPower(0);
                left_front_drive.setPower(0);
                right_back_drive.setPower(0);
                right_front_drive.setPower(0);

                // Turn off RUN_TO_POSITION
                left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                if (isBack) {
                    ultrasonicValue = (double)(ultrasonicBack.sampleDistance() / 2.54);
                } else {
                    ultrasonicValue = (double)(ultrasonicFront.sampleDistance() / 2.54);
                }

                double oldUltraError = ultraError;
                ultraError = distance - ultrasonicValue;




            }



        }
    }


    public void UltrasonicIterativeGyroDrive(double speed,
                                    double distance,
                                    double angle,
                                    boolean steeringToggle,
                                    double UltraTolerance,
                                    boolean isBack) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;



        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            double ultrasonicValue;

            //Determine whether to use back or front and then convert to freedom units

            if (isBack) {

                //use the back sensor
                ultrasonicValue = (double)(ultrasonicBack.sampleDistance() / 2.54);

            } else {

                //use the front sensor
                ultrasonicValue = (double)(ultrasonicFront.sampleDistance() / 2.54);

            }



            //take the first calculation of the distance error
            double ultraError = distance - ultrasonicValue;

            while (linearOpMode.opModeIsActive() && java.lang.Math.abs(ultraError) > UltraTolerance) {

                moveCounts = 0;

                double oldUltraError = ultraError;
                ultraError = distance - ultrasonicValue;



                if (java.lang.Math.abs(ultraError) > 5 ) {

                    if (ultraError > 5){

                        moveCounts = (int) (5 * COUNTS_PER_INCH);

                    } else if (ultraError < -5){

                        moveCounts = (int) (-5 * COUNTS_PER_INCH);

                    }



                } else if (java.lang.Math.abs(ultraError) < 5 && java.lang.Math.abs(ultraError) > UltraTolerance) {

                    moveCounts = (int)(ultraError * COUNTS_PER_INCH);

                }



                // Determine new target position, and pass to motor controller

                newLeftTarget = left_back_drive.getCurrentPosition() + moveCounts;
                newRightTarget = right_back_drive.getCurrentPosition() + moveCounts;


                // Set Target and Turn On RUN_TO_POSITION
                left_back_drive.setTargetPosition(newLeftTarget);
                left_front_drive.setTargetPosition(newLeftTarget);
                right_back_drive.setTargetPosition(newRightTarget);
                right_front_drive.setTargetPosition(newRightTarget);

                left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                left_back_drive.setPower(speed);
                left_front_drive.setPower(speed);
                right_back_drive.setPower(speed);
                right_front_drive.setPower(speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (linearOpMode.opModeIsActive() && (Math.abs(newLeftTarget - left_back_drive.getCurrentPosition()) > ENCODER_THRESHOLD || Math.abs(newRightTarget - right_back_drive.getCurrentPosition()) > ENCODER_THRESHOLD)) {
                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance > 0) {
                        steer *= -1.0;
                    }

                    if (steeringToggle) {
                        leftSpeed = speed - steer;
                        rightSpeed = speed + steer;
                    } else {
                        leftSpeed = speed;
                        rightSpeed = speed;
                    }

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    left_back_drive.setPower(-leftSpeed);
                    left_front_drive.setPower(-leftSpeed);
                    right_back_drive.setPower(-rightSpeed);
                    right_front_drive.setPower(-rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Actual", "%7d:%7d", left_back_drive.getCurrentPosition(),
                            right_back_drive.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", -leftSpeed, -rightSpeed);
                    telemetry.update();
                }

                // Stop all motion;
                left_back_drive.setPower(0);
                left_front_drive.setPower(0);
                right_back_drive.setPower(0);
                right_front_drive.setPower(0);

                // Turn off RUN_TO_POSITION
                left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                if (isBack) {
                    ultrasonicValue = (double)(ultrasonicBack.sampleDistance() / 2.54);
                } else {
                    ultrasonicValue = (double)(ultrasonicFront.sampleDistance() / 2.54);
                }




            }



        }
    }
}