package org.firstinspires.ftc.teamcode.subsystems.Driving;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoManagementV2 {
    CRServo glyph1;
    CRServo glyph2;
    CRServo glyph3, glyph4;
    CRServo jewelHinge;
    CRServo relicGrab;
    CRServo relicRot;
    CRServo jewelKnock;
    CRServo intake1;
    CRServo intake2;
    CRServo intake3;
    CRServo intake4;

    public boolean clawOpen1 = true;
    public boolean enableClaw1 = true;

    public boolean clawOpen2 = true;
    public boolean enableClaw2 = true;

    boolean jewelUp = true;

    public ServoManagementV2(HardwareMap hardwareMap) {
        //Hardware Map Naming
        glyph1 = hardwareMap.crservo.get("glyph1");
        glyph2 = hardwareMap.crservo.get("glyph2");
        glyph3 = hardwareMap.crservo.get("glyph3");
        glyph4 = hardwareMap.crservo.get("glyph4");
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        intake3 = hardwareMap.crservo.get("intake3");
        intake4 = hardwareMap.crservo.get("intake4");

        jewelHinge = hardwareMap.crservo.get("jewel_arm");
        jewelKnock = hardwareMap.crservo.get("jewel_knock");


        //Define Servo Direction
        jewelHinge.setDirection(CRServo.Direction.FORWARD);
        jewelKnock.setDirection(CRServo.Direction.FORWARD);

        glyph1.setDirection(CRServo.Direction.REVERSE);
        glyph2.setDirection(CRServo.Direction.FORWARD);
        glyph3.setDirection(CRServo.Direction.REVERSE);
        glyph4.setDirection(CRServo.Direction.FORWARD);

        intake1.setDirection(CRServo.Direction.REVERSE);
        intake2.setDirection(CRServo.Direction.FORWARD);
        intake3.setDirection(CRServo.Direction.REVERSE);
        intake4.setDirection(CRServo.Direction.FORWARD);

        relicGrab = hardwareMap.crservo.get("relicGrab");
        relicRot = hardwareMap.crservo.get("relicRot");


    }

    //Start with Jewel Up and Claw Open
    public void init() {
        glyph1.setPower(0.5);
        glyph2.setPower(0.5);
        glyph3.setPower(0.5);
        glyph4.setPower(0.5);

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
            glyph1.setPower(-.3);
            glyph2.setPower(-.3);
            clawOpen1 = false;
        }
        else if ((!clawOpen1)&&(enableClaw1)) {
            glyph1.setPower(0.15);
            glyph2.setPower(0.15);
            clawOpen1 = true;
        }
        enableClaw1 = false;
    }

    public void toggleClaw2() {
        if ((clawOpen2)&&(enableClaw2)) {
            glyph3.setPower(-.3);
            glyph4.setPower(-.3);
            clawOpen2 = false;
        }
        else if ((!clawOpen2)&&(enableClaw2)) {
            glyph3.setPower(0.15);
            glyph4.setPower(0.15);
            clawOpen2 = true;
        }
        enableClaw2 = false;
    }

    //Close Claw
    public void closeClaw() {
            glyph1.setPower(-.3);
            glyph2.setPower(-.3);
            glyph3.setPower(-.3);
            glyph4.setPower(-.3);
            clawOpen1 = false;
            clawOpen2 = false;
    }

    public void slightClaw() {
            glyph1.setPower(0);
            glyph2.setPower(0);
            glyph3.setPower(0);
            glyph4.setPower(0);
            clawOpen1 = true;
            clawOpen2 = true;
    }

    //Open Claw
    public void openClaw() {
            glyph1.setPower(0.15);
            glyph2.setPower(0.15);
            glyph3.setPower(.15);
            glyph4.setPower(.15);
            clawOpen1 = true;
            clawOpen2 = true;
    }

    public void clawIntake() {
        intake1.setPower(.75);
        intake2.setPower(.75);
        intake3.setPower(.75);
        intake4.setPower(.75);
    }
    public void clawEject() {
        intake1.setPower(-.75);
        intake2.setPower(-.75);
        intake3.setPower(-.75);
        intake4.setPower(-.75);
    }
    public void clawStop() {
        intake1.setPower(0);
        intake2.setPower(0);
        intake3.setPower(0);
        intake4.setPower(0);
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