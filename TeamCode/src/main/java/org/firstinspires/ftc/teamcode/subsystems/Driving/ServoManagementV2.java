package org.firstinspires.ftc.teamcode.subsystems.Driving;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoManagementV2 {
    CRServo clawTR;
    CRServo clawBR;
    CRServo clawTL, clawBL;
    CRServo jewelHinge;
    CRServo relicGrab;
    CRServo relicRot;
    CRServo jewelKnock;
    CRServo intakeTR;
    CRServo intakeBR;
    CRServo intakeTL;
    CRServo intakeBL;

    public boolean clawOpen1 = true;
    public boolean enableClaw1 = true;

    public boolean clawOpen2 = true;
    public boolean enableClaw2 = true;

    boolean jewelUp = true;

    public ServoManagementV2(HardwareMap hardwareMap) {
        //Hardware Map Naming
        clawTR = hardwareMap.crservo.get("clawTR");
        clawBR = hardwareMap.crservo.get("clawBR");
        clawTL = hardwareMap.crservo.get("clawTL");
        clawBL = hardwareMap.crservo.get("clawBL");
        intakeTR = hardwareMap.crservo.get("intakeTR");
        intakeBR = hardwareMap.crservo.get("intakeBR");
        intakeTL = hardwareMap.crservo.get("intakeTL");
        intakeBL = hardwareMap.crservo.get("intakeBL");

        jewelHinge = hardwareMap.crservo.get("jewel_arm");
        jewelKnock = hardwareMap.crservo.get("jewel_knock");


        //Define Servo Direction
        jewelHinge.setDirection(CRServo.Direction.FORWARD);
        jewelKnock.setDirection(CRServo.Direction.FORWARD);

        clawTR.setDirection(CRServo.Direction.REVERSE);
        clawBR.setDirection(CRServo.Direction.FORWARD);
        clawTL.setDirection(CRServo.Direction.FORWARD);
        clawBL.setDirection(CRServo.Direction.REVERSE);

        intakeTR.setDirection(CRServo.Direction.REVERSE);
        intakeBR.setDirection(CRServo.Direction.FORWARD);
        intakeTL.setDirection(CRServo.Direction.FORWARD);
        intakeBL.setDirection(CRServo.Direction.REVERSE);

        relicGrab = hardwareMap.crservo.get("relicGrab");
        relicRot = hardwareMap.crservo.get("relicRot");


    }

    //Start with Jewel Up and Claw Open
    public void init() {
        clawTR.setPower(0.5);
        clawBR.setPower(0.5);
        clawTL.setPower(0.5);
        clawBL.setPower(0.5);

        jewelHinge.setPower(0.6);
        jewelKnock.setPower(0);

        relicRot.setPower(.75);
        jewelUp = true;
    }

    //Lower Jewel Arm
    public void lowerJewel() {
        if (jewelUp) {
            jewelKnock.setPower(0);
            jewelHinge.setPower(-0.4);
            jewelUp = false;
        }
    }

    //Raise Jewel Arm
    public void raiseJewel() {
        if (!jewelUp) {
            jewelKnock.setPower(0);
            jewelHinge.setPower(0.5);
            jewelUp = true;
        }
    }

    //Knock Jewel
    public void knockJewel(double power){
        jewelKnock.setPower(power);
    }

    public void toggleClaw1() {
        if ((clawOpen1)&&(enableClaw1)) {
            clawTR.setPower(-.3);
            clawTL.setPower(-.3);
            clawOpen1 = false;
        }
        else if ((!clawOpen1)&&(enableClaw1)) {
            clawTR.setPower(0.15);
            clawTL.setPower(0.15);
            clawOpen1 = true;
        }
        enableClaw1 = false;
    }

    public void toggleClaw2() {
        if ((clawOpen2)&&(enableClaw2)) {
            clawBR.setPower(-.3);
            clawBL.setPower(-.3);
            clawOpen2 = false;
        }
        else if ((!clawOpen2)&&(enableClaw2)) {
            clawBR.setPower(0.15);
            clawBL.setPower(0.15);
            clawOpen2 = true;
        }
        enableClaw2 = false;
    }

    //Close Claw
    public void closeClaw() {
            clawTR.setPower(-.3);
            clawBR.setPower(-.3);
            clawTL.setPower(-.3);
            clawBL.setPower(-.3);
            clawOpen1 = false;
            clawOpen2 = false;
    }

    public void slightClaw() {
            clawTR.setPower(0);
            clawBR.setPower(0);
            clawTL.setPower(0);
            clawBL.setPower(0);
            clawOpen1 = true;
            clawOpen2 = true;
    }

    //Open Claw
    public void openClaw() {
            clawTR.setPower(0.15);
            clawBR.setPower(0.15);
            clawTL.setPower(0.15);
            clawBL.setPower(0.15);
            clawOpen1 = true;
            clawOpen2 = true;
    }

    public void clawIntake() {
        intakeTR.setPower(.75);
        intakeBR.setPower(.75);
        intakeTL.setPower(.75);
        intakeBL.setPower(.75);
    }
    public void clawEject() {
        intakeTR.setPower(-.75);
        intakeBR.setPower(-.75);
        intakeTL.setPower(-.75);
        intakeBL.setPower(-.75);
    }
    public void clawStop() {
        intakeTR.setPower(0);
        intakeBR.setPower(0);
        intakeTL.setPower(0);
        intakeBL.setPower(0);
    }


    public void openRelic() {
        relicGrab.setPower(-.75);
    }

    public void closeRelic() {
        relicGrab.setPower(.75);
    }

    public void  relicRelease() {
        relicGrab.setPower(-.5);
    }

    public void rotateUp() {
        relicRot.setPower(-.75);
    }

    public void rotateDown() {
        relicRot.setPower(.75);
    }

    public void getRotate() {
        relicRot.getPower();
    }


}