package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.subsystems.PID_Library;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.subsystems.ServoManagementV2;

/**
 * Created by Evan McLoughlin on 12/14/2017.
 */

@Autonomous(name = "Festus Auto Red 1 Balance", group = "Festus")
public class Festus_Auto_Red_1_S_Balance extends LinearOpMode {
    DcMotor liftMotor;
    ColorSensor color;


    RobotVision vMod;
    ServoManagementV2 srvo;
    PID_Library enc;

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

        enc = new PID_Library(hardwareMap, telemetry,this);

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(true);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //-----------------------------------------=+(Hardware Map)+=-----------------------------------------\\

        //-------------------------------------=+(Initialization Config)+=------------------------------------\\
        //srvo.rotateRelic(0.5);
        srvo.raiseJewel();
        telemetry.addData(">", "Robot Ready!");
        telemetry.update();
        waitForStart();
        //-------------------------------------=+(Initialization Config)+=------------------------------------\\

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //-----------------------------------------=+(Autonomous)+=-------------------------------------------\\

            //Step 1: Close The Claw
            srvo.closeClaw();
            waitFor(800);

            //Step 2: Lift Cube
            liftMotor.setPower(-0.3);
            waitFor(500);

            liftMotor.setPower(0);

            //Step 3: Lower Jewel Arm
            srvo.lowerJewel();
            waitFor(2700);

            //Reset time for Jewel method
            etime.reset();
            while ((etime.time() < 2)&&(opModeIsActive())) {
                //Step 4: Jewel Knock Method
                if (color.red() > color.blue()) {//if red
                    //Knock off Blue
                    srvo.knockJewel(0.5);
                    waitFor(1500);
                    srvo.knockJewel(0);
                    waitFor(1500);

                    //Bring up Arm
                    srvo.raiseJewel();
                    waitFor(1000);
                    break;
                } else if (color.red() < color.blue()) {//if blue
                    //Knock off Blue
                    srvo.knockJewel(-0.5);
                    waitFor(1500);
                    srvo.knockJewel(0);
                    waitFor(1500);

                    //Bring up Arm
                    srvo.raiseJewel();
                    waitFor(1000);
                    break;
                }
                telemetry.addData("RED",color.red());
                telemetry.addData("GREEN",color.green());
                telemetry.addData("BLUE",color.blue());
                telemetry.update();
            }
            //Bring up Arm
            srvo.raiseJewel();
            waitFor(1500);

            //Step 6: Vision method
            //Get Position from Vision
            // 0 LEFT
            // 1 CENTER
            // 2 RIGHT

            //Reset time for Vision method
            etime.reset();
            while ((etime.time() < 2)&&(opModeIsActive())) {
                vMod.getVuMark();
                if (vMod.vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark Status - ", "Left");
                    position = 0;
                    break;
                } else if (vMod.vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark Status - ", "Center");
                    position = 1;
                    break;
                } else if (vMod.vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark Status - ", "Right");
                    position = 2;
                    break;
                }
                telemetry.update();
                idle();
            }
            //If 2 Second timeout failed use 1 (CENTER)
            if (vMod.vuMark == RelicRecoveryVuMark.UNKNOWN) {
                position = 1;
            }
            //Display Position
            telemetry.addData("Position:", position);
            telemetry.update();
            waitFor(2000);

            //AUTO CALIBRATION
            //from this point and below to easily calibrate auto use the encoderTest to find the distance between the left/right columns relative to center
            //then all you need to do is make sure center works and use the differences to have left and right working!!

            double centerPosition = 36;
            double offset = 0;
            if (position == 0) { //Left
                offset = 7.5;
            }else if (position == 2) { //Right
                offset = -7.5;
            }
            double distance = centerPosition+offset;

            //Step 7: Drive to Appropriate Column
            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, -distance, 0,false);
            waitFor(2000);

            //New step: reposition w/ balancing stone

            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, distance - 9 - 12, 0,false);
            waitFor(500);

            //New step 2: Drive back to desired position
            //12 inches is subtracted from distance because this is the offset from the center to edge of the balance board
            //9 inches is again subtracted from distance because this is the offset from the edge of the balance board to the center of the bot
            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, -distance + 9 + 12, 0,false);
            waitFor(500);

            //Step 8: Turn 90 Degrees
            enc.gyroTurn(enc.TURN_SPEED, -90);
            waitFor(500);
            enc.gyroHold(enc.TURN_SPEED, -90, 1.5);

            //Step 9: Open Claw
            srvo.openClaw();

            //Step 10: Push Glyph into Column
            waitFor(500);
            enc.gyroDrive(enc.DRIVE_SPEED, -8, -90,true);
            enc.gyroDrive(enc.DRIVE_SPEED, 5, -90,true);

            //Step 11: Turn around towards field
            enc.gyroTurn(enc.TURN_SPEED, 90);

            //NEW CODE TO GET SECOND GLYPH //

            //Slight Claw
            srvo.slightClaw();
            waitFor(500);

            //Drive to Glyph
            enc.gyroDrive(enc.DRIVE_SPEED, -26, 90,true);
            waitFor(500);

            //Close and Lift
            srvo.closeClaw();
            waitFor(500);
            liftMotor.setPower(-0.8);
            waitFor(1500);
            liftMotor.setPower(0);

            //Turn Around
            enc.gyroTurn(enc.TURN_SPEED, -90);
            waitFor(500);

            //Drive to Column
            enc.gyroDrive(enc.DRIVE_SPEED, -28, -90,true);
            waitFor(500);

            //Back Off
            srvo.openClaw();
            waitFor(500);
            enc.gyroDrive(enc.DRIVE_SPEED, 3, -90,true);
            waitFor(500);

            //END NEW CODE TO GET SECOND GLYPH //

            //End While Loop
            break;
        }
        stop();
    }
}