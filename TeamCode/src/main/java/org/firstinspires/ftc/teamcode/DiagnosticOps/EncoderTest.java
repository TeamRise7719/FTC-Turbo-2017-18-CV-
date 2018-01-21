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

    public void init(){
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
        setMotorToEnc();
    }


    private void setMotorToEnc(){
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void loop(){
        telemetry.addData("encoder 1", left_back_drive.getCurrentPosition());
        telemetry.addData("encoder 2", left_front_drive.getCurrentPosition());
        telemetry.addData("encoder 3", right_back_drive.getCurrentPosition());
        telemetry.addData("encoder 4", right_front_drive.getCurrentPosition());
        telemetry.update();
    }

}
