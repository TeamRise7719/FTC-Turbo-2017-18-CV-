package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.subsystems.Driving.PID_Library;
import org.firstinspires.ftc.teamcode.subsystems.Sensing.RobotVision;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;

/**
 * Created by seancardosi on 12/5/17.
 */

@Autonomous(name = "Festus_Auto_Blue_2_PID", group = "Festus")
public class Festus_Auto_Blue_2_PID extends LinearOpMode {
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
        enc.init();

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");
        color.enableLed(true);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //-----------------------------------------=+(Hardware Map)+=-----------------------------------------\\


        //-------------------------------------=+(Initialization Config)+=------------------------------------\\
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
            waitFor(1000);

            //Step 2: Lift Cube
            liftMotor.setPower(-0.1);
            waitFor(1000);
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
                    srvo.knockJewel(-0.5);
                    waitFor(1500);
                    srvo.knockJewel(0);
                    //Bring up Arm
                    srvo.raiseJewel();
                    waitFor(1000);
                    break;
                } else if (color.red() < color.blue()) {//if blue
                    //Knock off Blue
                    srvo.knockJewel(0.5);
                    waitFor(1500);
                    srvo.knockJewel(0);
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
                    position = 2;
                    break;
                } else if (vMod.vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark Status - ", "Center");
                    position = 1;
                    break;
                } else if (vMod.vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark Status - ", "Right");
                    position = 0;
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

            //Step 7: Drive off Balancing Stone
            enc.gyroDrive(enc.DRIVE_SPEED_SLOW, -22.5, 0,false);
            waitFor(1500);

            //Step 8: Turn Back Straight
           // enc.gyroHold(enc.TURN_SPEED, 0, 1.5);
           // waitFor(500);

            //Step 9: Turn Towards Columns
            enc.gyroTurn(enc.TURN_SPEED, 90);
            waitFor(1000);

           // enc.gyroHold(enc.TURN_SPEED, 90, 1000);
            //waitFor(1000);


            //from this point and below to easily calibrate auto use the EncoderTest to find the distance between the left/right columns relative to center
            //then all you need to do is make sure center works and use the differences to have left and right working!!

            double centerPosition = 26;
            double offset = 0;
            if (position == 0) { //Right
                offset = 7;
            }else if (position == 2) { //Left
                offset = -7;
            }
            double distance = centerPosition+offset;

            //Step 10: Drive to Appropriate Column
            //enc.gyroDrive(enc.DRIVE_SPEED_SLOW, distance, 90,false);
            enc.UltrasonicGyroDrive(distance, 90,false, 0.5, true, 5);
            waitFor(2000);

            //Step 11: Turn back to 180 Degrees
            enc.gyroTurn(enc.TURN_SPEED, 180);
            waitFor(500);

            //Step 12: Open Claw
            srvo.openClaw();

            //Step 10: Push Glyph into Column
            waitFor(500);
            enc.gyroDrive(.45, 8, 180,false);
            enc.gyroDrive(enc.DRIVE_SPEED, -6, 180, false);

            //Step 11: Turn around towards field
            enc.gyroTurn(enc.TURN_SPEED,  90);
            srvo.openClaw();

            //End While Loop
            break;
        }
        //Switch Autonomous to TeleOp
        AutoTransitioner.transitionOnStop(this, "EvanTeleOp");
    }
}
