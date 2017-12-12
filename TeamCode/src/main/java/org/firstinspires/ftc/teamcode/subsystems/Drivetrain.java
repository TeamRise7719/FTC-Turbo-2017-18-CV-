package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor left_back_drive;
    public DcMotor left_front_drive;
    public DcMotor right_back_drive;
    public DcMotor right_front_drive;
    public DcMotor liftMotor;
    public DcMotor winchMotor;

    public Drivetrain(HardwareMap hardwareMap){
        //configuring the components
        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchMotor = hardwareMap.dcMotor.get("winch");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_back_drive = hardwareMap.dcMotor.get("1");
        left_front_drive = hardwareMap.dcMotor.get("2");
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;

        right_back_drive = hardwareMap.dcMotor.get("3");
        right_front_drive = hardwareMap.dcMotor.get("4");
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void raiseLift(double power){
        liftMotor.setPower(-power);
    }

    public void stopLift(){
        liftMotor.setPower(0);
    }

    public void lowerLift(double power){
        liftMotor.setPower(power);
    }

    public void extendWinch(double power) { winchMotor.setPower(power);}

    public void retractWinch(double power) { winchMotor.setPower(-power);}

    public void stopWinch() { winchMotor.setPower(0);}

    public void drive(Gamepad gamepad1) {

        //----------------------------------------------=+(DRIVETRAIN)+=----------------------------------------------\\



//        double r_trigger_pow = gamepad1.right_trigger;
//        r_trigger_pow = (math.asin((r_trigger_pow * r_trigger_pow)) * 2) / math.PI;
//
//        double l_trigger_pow = gamepad1.right_trigger;
//        l_trigger_pow = (math.asin((l_trigger_pow * l_trigger_pow)) * 2) / math.PI;

//        float LFspeed = gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger;//+
//        float LBspeed = gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger;//-
//        float RFspeed = gamepad1.right_stick_y - gamepad1.left_trigger + gamepad1.right_trigger;//-
//        float RBspeed = gamepad1.right_stick_y + gamepad1.left_trigger - gamepad1.right_trigger;//+

        float LFspeed = gamepad1.left_stick_y;//+
        float LBspeed = gamepad1.left_stick_y;//-
        float RFspeed = gamepad1.right_stick_y;//-
        float RBspeed = gamepad1.right_stick_y;//+


        if (gamepad1.y) {
            left_back_drive.setPower(-0.6);
            left_front_drive.setPower(-0.6);
            right_back_drive.setPower(-0.6);
            right_front_drive.setPower(-0.6);
        } else if (gamepad1.a){
            left_back_drive.setPower(0.6);
            left_front_drive.setPower(0.6);
            right_back_drive.setPower(0.6);
            right_front_drive.setPower(0.6);
        }

//        LFspeed = Range.clip(LFspeed, -1, 1);
//        LBspeed = Range.clip(LBspeed, -1, 1);
//        RFspeed = Range.clip(RFspeed, -1, 1);
//        RBspeed = Range.clip(RBspeed, -1, 1);

        if(Math.abs(gamepad1.left_trigger)>.2){
            left_back_drive.setPower(LBspeed/2);
            left_front_drive.setPower(LFspeed/2);
            right_back_drive.setPower(RBspeed/2);
            right_front_drive.setPower(RFspeed/2);        }
        else {
            left_back_drive.setPower(LBspeed);
            left_front_drive.setPower(LFspeed);
            right_back_drive.setPower(RBspeed);
            right_front_drive.setPower(RFspeed);
        }

        //----------------------------------------------=+(DRIVETRAIN)+=----------------------------------------------\\
    }

}