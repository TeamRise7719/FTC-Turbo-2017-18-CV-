package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Sensing.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.subsystems.Sensing.RobotVision;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;

/**
 * Created by Evan on 12/4/2017.
 */
@Autonomous(name = "Festus_Auto_Blue_1_S", group = "Festus")
public class Festus_Auto_Blue_1_S extends LinearOpMode {
    ColorSensor color;
    I2CXL ultrasonicFront;

    RobotVision vMod;
    ServoManagementV2 srvo;
    SeansEncLibrary enc;

    ElapsedTime etime = new ElapsedTime();

    int position = 0;

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    public void runOpMode() throws InterruptedException {

        //-----------------------------------------=+(Hardware Map)+=-----------------------------------------\\
        srvo = new ServoManagementV2(hardwareMap);
        srvo.init();


        vMod = new RobotVision(hardwareMap, telemetry);
        vMod.init();

        enc = new SeansEncLibrary(hardwareMap, telemetry,this);
        enc.init();
        enc.resetGlyphRotateMotor();

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(true);

        ultrasonicFront = hardwareMap.get(I2CXL.class, "ultsonFront");
        ultrasonicFront.initialize();

        //-----------------------------------------=+(Hardware Map)+=-----------------------------------------\\

        //-------------------------------------=+(Initialization Config)+=------------------------------------\\
        srvo.raiseJewel();
        telemetry.addData(">", "Robot Ready!");
        telemetry.update();



        while (!isStarted()&&opModeIsActive()){
                vMod.getVuMark();
                if (vMod.vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark Status - ", "Left");
                    position = 2;
                } else if (vMod.vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark Status - ", "Center");
                    position = 1;
                } else if (vMod.vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark Status - ", "Right");
                    position = 0;
                }
                telemetry.update();
                idle();
            if (vMod.vuMark == RelicRecoveryVuMark.UNKNOWN) {
                position = 1;
            }
            //Display Position
            telemetry.addData("Position:", position);
            telemetry.update();
        }
        //-------------------------------------=+(Initialization Config)+=------------------------------------\\

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //-----------------------------------------=+(Autonomous)+=-------------------------------------------\\

            //Step 1: Close The Claw
            srvo.closeClaw();
            waitFor(250);

            //Step 2: Lift Cube
            enc.moveLiftTime(-0.25,1.1);

            //Step 3: Lower Jewel Arm
            srvo.lowerJewel();
            waitFor(500);

            //Reset time for Jewel method
            etime.reset();
            while ((etime.time() < 1)&&(opModeIsActive())) {
                //Step 4: Jewel Knock Method
                if (color.red() > color.blue()) {//if red
                    //Knock off Blue
                    srvo.knockJewel(-0.5);
                    waitFor(500);
                    srvo.knockJewel(0);

                    //Bring up Arm
                    srvo.raiseJewel();
                    waitFor(500);
                    break;
                } else if (color.red() < color.blue()) {//if blue
                    //Knock off Blue
                    srvo.knockJewel(0.5);
                    waitFor(500);
                    srvo.knockJewel(0);

                    //Bring up Arm
                    srvo.raiseJewel();
                    waitFor(500);
                    break;
                }
                telemetry.addData("RED",color.red());
                telemetry.addData("GREEN",color.green());
                telemetry.addData("BLUE",color.blue());
                telemetry.update();
            }
            //Bring up Arm
            srvo.raiseJewel();
            //waitFor(500);

            //Step 6: Vision method
            //Get Position from Vision
            // 0 LEFT
            // 1 CENTER
            // 2 RIGHT

            //Reset time for Vision method
//            etime.reset();
//            while ((etime.time() < 0.5)&&(opModeIsActive())) {
//                vMod.getVuMark();
//                if (vMod.vuMark == RelicRecoveryVuMark.LEFT) {
//                    telemetry.addData("VuMark Status - ", "Left");
//                    position = 2;
//                    break;
//                } else if (vMod.vuMark == RelicRecoveryVuMark.CENTER) {
//                    telemetry.addData("VuMark Status - ", "Center");
//                    position = 1;
//                    break;
//                } else if (vMod.vuMark == RelicRecoveryVuMark.RIGHT) {
//                    telemetry.addData("VuMark Status - ", "Right");
//                    position = 0;
//                    break;
//                }
//                telemetry.update();
//                idle();
//            }
//            //If 0.5 Second timeout failed use 1 (CENTER)
//            if (vMod.vuMark == RelicRecoveryVuMark.UNKNOWN) {
//                position = 1;
//            }
//            //Display Position
//            telemetry.addData("Position:", position);
//            telemetry.update();


            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, -24, 0,false);
            waitFor(500);
//            enc.gyroTurn(enc.TURN_SPEED, 180);
//            waitFor(500);

            //AUTO CALIBRATION
            //from this point and below to easily calibrate auto use the encoderTest to find the distance between the left/right columns relative to center
            //then all you need to do is make sure center works and use the differences to have left and right working!!

            //Step 7: Drive to Appropriate Column
            double distance;

            if(ultrasonicFront.getDistance()>1) {
                double centerPosition = 50.5;
                double offset = 0;

                if (position == 0) { //Right
                    offset = 7;
                } else if (position == 2) { //Left
                    offset = -7;
                }

                distance = (centerPosition + offset) - (ultrasonicFront.getDistance() / 2.54);

                if (distance > 13) {
                    distance = 11.8;
                }
            }
            else{
                double centerPosition = 37.5;
                double offset = 0;

                if (position == 0) { //Right
                    offset = 7;
                } else if (position == 2) { //Left
                    offset = -7;
                }

                distance = centerPosition + offset;
            }

            telemetry.addData("Distance", distance);
            telemetry.update();
            waitFor(250);

            enc.gyroDrive(enc.DRIVE_SPEED_SLOW,distance,0,false);
            waitFor(500);

            //Step 8: Turn 90 Degrees
            enc.gyroTurn(enc.TURN_SPEED, -90);
            waitFor(500);

            //Step 9: Open Claw
            srvo.openClaw();

            //Step 10: Push Glyph into Column
            waitFor(500);
            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, 8, -90,false);
            enc.gyroDrive(enc.DRIVE_SPEED, -6, -90,false);

            //Step 11: Turn around towards field
            enc.gyroTurn(enc.TURN_SPEED, 90);

            //NEW CODE TO GET SECOND GLYPH //

            enc.moveLiftTime(0.1,1.1);
            srvo.slightClaw();
            waitFor(1000);
            enc.moveLiftTime(0.0, 0.1);

            //Drive to Glyph+9*
            enc.gyroDrive(0.8, 26, 90,false);
            waitFor(250);

            //Close and Lift
            srvo.closeClaw();
            waitFor(250);
            enc.moveLiftTime(-0.8,1.5);

            //Turn Around
            enc.gyroTurn(enc.TURN_SPEED, -90);
            waitFor(250);

            //Drive to Column
            enc.gyroDrive(enc.DRIVE_SPEED, 28, -90,false);
            waitFor(250);

            //Back Off
            srvo.openClaw();
            waitFor(250);
            enc.gyroDrive(enc.DRIVE_SPEED, -6, -90,false);
            waitFor(250);

            //END NEW CODE TO GET SECOND GLYPH //

            //End While Loop
            break;
        }
        //Switch Autonomous to TeleOp
        srvo.clawStop1();
        srvo.clawStop2();
        AutoTransitioner.transitionOnStop(this, "EvanTeleOp");
    }
}
