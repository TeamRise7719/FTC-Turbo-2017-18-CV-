package org.firstinspires.ftc.teamcode.subsystems.Driving;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Admin on 1/27/2018.
 */

public class FestusLift {

    DcMotor liftMotorL, liftMotorR;
    public DcMotor glyphRotate;

    Telemetry telemetry;

    public boolean glyphRotated = false;
    public boolean glyphReset = false;

    public FestusLift(HardwareMap hardwareMap, Telemetry tel) {
        liftMotorL = hardwareMap.dcMotor.get("liftL");
        liftMotorR = hardwareMap.dcMotor.get("liftR");
        glyphRotate = hardwareMap.dcMotor.get("glyphRotate");

        telemetry = tel;
    }

    public void init(){
        liftMotorL.setDirection(DcMotor.Direction.FORWARD);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphRotate.setDirection(DcMotor.Direction.FORWARD);
        glyphRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void resetGlyphRotateMotor(){
        glyphRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphReset = true;
    }

    public void setLiftPower(double power){
        liftMotorL.setPower(power);
        liftMotorR.setPower(power);
    }

    public void moveLiftTime(double power, double time,LinearOpMode linearOpMode) {
        ElapsedTime etime = new ElapsedTime();
        etime.reset();
        while ((etime.time() < time) && (linearOpMode.opModeIsActive())) {
            liftMotorL.setPower(power);
            liftMotorR.setPower(power);
        }
        liftMotorL.setPower(0);
        liftMotorR.setPower(0);
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

    public void rotateGlyphDown() {
        glyphRotate.setTargetPosition(0);
        glyphRotate.setPower(.25);
        glyphRotated = false;
    }
    public void rotateGlyphUp() {
        glyphRotate.setTargetPosition(560);
        glyphRotate.setPower(-.25);
        glyphRotated = true;
    }
}
