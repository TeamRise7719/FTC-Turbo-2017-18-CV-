package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by seancardosi on 10/25/17.
 */
@TeleOp(name = "PIDValues", group = "Festus")
public class PIDValues extends OpMode {

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
    }


    public void loop(){
        DcMotorControllerEx motorControllerExLB = (DcMotorControllerEx)left_back_drive.getController();
        DcMotorControllerEx motorControllerExLF = (DcMotorControllerEx)left_front_drive.getController();
        DcMotorControllerEx motorControllerExRB = (DcMotorControllerEx)right_back_drive.getController();
        DcMotorControllerEx motorControllerExRF = (DcMotorControllerEx)right_front_drive.getController();

        int motorIndexLB = ((DcMotorEx)left_back_drive).getPortNumber();
        PIDCoefficients pidOrigLB = motorControllerExLB.getPIDCoefficients(motorIndexLB, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorIndexLF = ((DcMotorEx)left_front_drive).getPortNumber();
        PIDCoefficients pidOrigLF= motorControllerExLF.getPIDCoefficients(motorIndexLF, DcMotor.RunMode.RUN_USING_ENCODER);

        int motorIndexRB = ((DcMotorEx)right_back_drive).getPortNumber();
        PIDCoefficients pidOrigRB = motorControllerExRB.getPIDCoefficients(motorIndexRB, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorIndexRF = ((DcMotorEx)left_front_drive).getPortNumber();
        PIDCoefficients pidOrigRF= motorControllerExRF.getPIDCoefficients(motorIndexRF, DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P,I,D LB", "%.04f, %.04f, %.0f",
                pidOrigLB.p, pidOrigLB.i, pidOrigLB.d);
        telemetry.addData("P,I,D LF", "%.04f, %.04f, %.0f",
                pidOrigLF.p, pidOrigLF.i, pidOrigLF.d);
        telemetry.addData("P,I,D TB", "%.04f, %.04f, %.0f",
                pidOrigRB.p, pidOrigRB.i, pidOrigRB.d);
        telemetry.addData("P,I,D RF", "%.04f, %.04f, %.0f",
                pidOrigRF.p, pidOrigRF.i, pidOrigRF.d);

        telemetry.update();
    }

}
