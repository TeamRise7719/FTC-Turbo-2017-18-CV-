package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by seancardosi on 10/25/17.
 */
@TeleOp(name = "EncoderTestTester", group = "Festus")
public class EncoderTest extends OpMode {

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;
    DcMotor glyphRotate;
    public void init(){
        glyphRotate = hardwareMap.dcMotor.get("glyphRotate");

        left_back_drive = hardwareMap.dcMotor.get("driveBL");
        left_front_drive = hardwareMap.dcMotor.get("driveFL");
        right_back_drive = hardwareMap.dcMotor.get("driveBR");
        right_front_drive = hardwareMap.dcMotor.get("driveFR");

        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);;

        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        glyphRotate.setDirection(DcMotor.Direction.FORWARD);
        glyphRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);;

        setMotorToEnc();
        resetGlyphRotateMotor();
    }


    public void resetGlyphRotateMotor(){
        glyphRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorToEnc(){
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void loop(){
        telemetry.addData("encoder LB", left_back_drive.getCurrentPosition());
        telemetry.addData("encoder LF", left_front_drive.getCurrentPosition());
        telemetry.addData("encoder RB", right_back_drive.getCurrentPosition());
        telemetry.addData("encoder RF", right_front_drive.getCurrentPosition());
        telemetry.addData("rotate", glyphRotate.getCurrentPosition());

        telemetry.update();
    }

}
