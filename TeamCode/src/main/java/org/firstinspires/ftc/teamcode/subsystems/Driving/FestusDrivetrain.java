package org.firstinspires.ftc.teamcode.subsystems.Driving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class FestusDrivetrain {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public boolean glyphRotated = false;
    public boolean glyphReset = false;

    private final DcMotor lf, lr, rf, rr;
    private DcMotor liftMotorL, liftMotorR;
    private DcMotor winchMotor;
    private DcMotor glyphRotate;


    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;

    public FestusDrivetrain(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        //configuring the components
        liftMotorL = hardwareMap.dcMotor.get("liftL");
        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorR = hardwareMap.dcMotor.get("liftR");
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchMotor = hardwareMap.dcMotor.get("winch");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphRotate = hardwareMap.dcMotor.get("glyphRotate");
        glyphRotate.setDirection(DcMotor.Direction.FORWARD);
        glyphRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lr = hardwareMap.dcMotor.get("driveBL");
        lf = hardwareMap.dcMotor.get("driveFL");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rr = hardwareMap.dcMotor.get("driveBR");
        rf = hardwareMap.dcMotor.get("driveFR");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imuINT");

        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void resetGlyphRotateMotor(){
        glyphRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphReset = true;
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _lf Left front motor
     * @param _lr Left rear motor
     * @param _rf Right front motor
     * @param _rr Right rear motor
     */
    public void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    public void setLiftPower(double power){
        liftMotorL.setPower(power);
        liftMotorR.setPower(power);
    }

    public void rotateGlyph() {
        if (glyphRotated) {
            glyphRotate.setTargetPosition(0);
            glyphRotate.setPower(.5);
            glyphRotated = false;
        }
        else if (!glyphRotated) {
            glyphRotate.setTargetPosition(560);
            glyphRotate.setPower(-.5);
            glyphRotated = true;
        }
    }

    public void winch(double power) { winchMotor.setPower(power);}

    public void drive(Gamepad gamepad1, Telemetry telemetry) {
        loop();

        telemetry.addData("RAW Gyro: ",getRawHeading());
        telemetry.addData("Heading: ",getHeading());
        telemetry.addData("Offset: ",headingOffset);

        telemetry.update();

        final double x = -gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;

        final double rotation = -(gamepad1.right_stick_x);
        final double direction = Math.atan2(x, y) + getHeading();
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        if (gamepad1.a){
            setMotors(lf / 2, lr / 2, rf / 2, rr / 2);
        } else {
            setMotors(lf, lr, rf, rr);
        }

    }
}