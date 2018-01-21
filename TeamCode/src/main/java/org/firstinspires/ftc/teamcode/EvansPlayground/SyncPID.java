package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Driving.SynchronousPID;

/**
 * Created by seancardosi on 10/25/17.
 */
@TeleOp(name = "SyncPID Test", group = "Festus")
public class SyncPID extends LinearOpMode {

    DcMotor left_back_drive;
    DcMotor left_front_drive;
    DcMotor right_back_drive;
    DcMotor right_front_drive;

    SynchronousPID left_back_PID;
    SynchronousPID left_front_PID;
    SynchronousPID right_back_PID;
    SynchronousPID right_front_PID;

    double p = 0.06;
    double i = 0;
    double d = 0;


    private void setMotorToEnc(){
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() throws InterruptedException {
        left_back_drive = hardwareMap.dcMotor.get("driveBL");
        left_front_drive = hardwareMap.dcMotor.get("driveFL");
        right_back_drive = hardwareMap.dcMotor.get("driveBR");
        right_front_drive = hardwareMap.dcMotor.get("driveFR");

        left_back_PID = new SynchronousPID(p,i,d);
        left_back_PID.setContinuous(true);
        left_back_PID.setInputRange(0.1,0.8);

        left_front_PID = new SynchronousPID(p,i,d);
        left_front_PID.setContinuous(true);
        left_front_PID.setInputRange(0.1,0.8);

        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);;

        right_back_PID = new SynchronousPID(p,i,d);
        right_back_PID.setContinuous(true);
        right_back_PID.setInputRange(0.1,0.8);

        right_front_PID = new SynchronousPID(p,i,d);
        right_front_PID.setContinuous(true);
        right_front_PID.setInputRange(0.1,0.8);

        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setMotorToEnc();

        telemetry.addData(">", "Robot Ready!");
        telemetry.update();
        waitForStart();

        double tolerance = 10;
        left_front_PID.setSetpoint(1128);
        left_back_PID.setSetpoint(1128);
        right_front_PID.setSetpoint(1128);
        right_back_PID.setSetpoint(1128);

        while((!left_front_PID.onTarget(tolerance))||(!right_front_PID.onTarget(tolerance))||(!left_back_PID.onTarget(tolerance))||(!right_back_PID.onTarget(tolerance))){
            left_front_drive.setPower(left_front_PID.calculate(left_front_drive.getCurrentPosition()));
            left_back_drive.setPower(left_back_PID.calculate(left_back_drive.getCurrentPosition()));
            right_front_drive.setPower(right_front_PID.calculate(right_front_drive.getCurrentPosition()));
            right_back_drive.setPower(right_back_PID.calculate(right_back_drive.getCurrentPosition()));

            telemetry.addData("encoder 1", left_back_drive.getCurrentPosition());
            telemetry.addData("encoder 2", left_front_drive.getCurrentPosition());
            telemetry.addData("encoder 3", right_back_drive.getCurrentPosition());
            telemetry.addData("encoder 4", right_front_drive.getCurrentPosition());
            telemetry.update();
        }
    }

}
