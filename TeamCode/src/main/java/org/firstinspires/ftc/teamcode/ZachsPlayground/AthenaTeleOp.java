package org.firstinspires.ftc.teamcode.ZachsPlayground;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by nonba on 12/14/2017.
 */

@Autonomous(name = "Athena_TeleOp", group = "AthenaTeleOp")

public class AthenaTeleOp  extends LinearOpMode {
    DcMotor liftMotor;
    DcMotor leftDrive;
    DcMotor rightDrive;

    CRServo glyph1;
    CRServo glyph2;

    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive = hardwareMap.dcMotor.get("1");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive = hardwareMap.dcMotor.get("2");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyph1 = hardwareMap.crservo.get("glyph1");
        glyph2 = hardwareMap.crservo.get("glyph2");

        glyph1.setPower(0.5);
        glyph2.setPower(0.5);

        waitForStart();

        while(opModeIsActive()) {
            leftDrive.setPower(gamepad1.left_stick_y);
            leftDrive.setPower(gamepad1.right_stick_x);
            if(gamepad1.y) {
                leftDrive.setPower(0.6);
            }
            else if(gamepad1.x) {
                leftDrive.setPower(-0.6);
            }

            if(gamepad1.x) {
                glyph1.setPower(0.5);
                glyph2.setPower(0.5);
            }
            else if(gamepad1.b) {
                glyph1.setPower(-.25);
                glyph2.setPower(-.25);
            }
            idle();
        }
    }

}
