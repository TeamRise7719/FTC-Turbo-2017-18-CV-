package org.firstinspires.ftc.teamcode.CVTestOpModes;

/**
 * Created by Evan McLoughlin on 12/12/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Field Oriented w/ CV", group = "Festus")
public class FieldOriented_CV_Test extends OpMode {
    private FestusDrivetrain robot;
    ServoManagementV2 srvo;
    private ElapsedTime runtime = new ElapsedTime();
    private CryptoboxDetector cryptoboxDetector = null;

    @Override
    public void init() {
        //Initialize robot
        robot = new FestusDrivetrain(hardwareMap, telemetry);



        //Initialize Servos
        srvo = new ServoManagementV2(hardwareMap);
        srvo.init();

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.downScaleFactor = 0.4;
        cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.SLOW;
        cryptoboxDetector.rotateMat = false;

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {

        //-------------------------------------------=+(Computer Vision)+=--------------------------------------------\\
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
        telemetry.addData("isColumnDetected ",  cryptoboxDetector.isColumnDetected());

        telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
        telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
        telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());
        //-------------------------------------------=+(Computer Vision)+=--------------------------------------------\\


        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
        robot.arcadeMode = true;
        robot.drive(gamepad1, telemetry);

//        //THIS MAY TOTALLY NOT WORK AT ALL
//        robot.rotate(gamepad1, telemetry);

        if (gamepad1.x) {
            robot.resetHeading();
        }

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\
        //Keep Jewel Arm Up and in starting position
        srvo.raiseJewel();
        srvo.knockJewel(0);
        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\


        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\
        if (gamepad2.b) {
            srvo.openClaw();
        } else if (gamepad2.x) {
            srvo.closeClaw();
        } else if (gamepad2.right_bumper) {
            srvo.slightClaw();
        }

        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\


        //----------------------------------------------=+(Relic)+=----------------------------------------------\\
        if (gamepad2.right_bumper) {
            srvo.openRelic();
        } else if (gamepad2.left_bumper) {
            srvo.closeRelic();
        }
        if (gamepad2.dpad_up) {
            srvo.rotateUp();
        } else if (gamepad2.dpad_down) {
            srvo.rotateDown();
        }


        robot.winch(gamepad2.left_stick_x);
        //----------------------------------------------=+(Relic)+=----------------------------------------------\\


        telemetry.addData("plz", "blease");
//        telemetry.addData("srvo",srvo.getRotate());


        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

        if (gamepad2.y) {
            robot.raiseLift(1);
        } else if (gamepad2.a) {
            robot.lowerLift(1);
        } else {
            robot.stopLift();
        }

        telemetry.addData("encoders", robot.liftMotor.getCurrentPosition());
        telemetry.update();


        //This is a very useful function, it just needs to be massively recalibrated to work with the current lift

//        if (robot.liftMotor.getCurrentPosition() < -100 || robot.liftMotor.getCurrentPosition() > 100) {
//
//            if (gamepad2.left_bumper) {
//
//                while (robot.liftMotor.getCurrentPosition() < -100 || robot.liftMotor.getCurrentPosition() > 100) {
//
//                    if (robot.liftMotor.getCurrentPosition() < -100){
//                        robot.raiseLift(.8);
//                    }
//
//                    else if (robot.liftMotor.getCurrentPosition() > 100) {
//                        robot.lowerLift(.8);
//
//                    }
//                }
//            }
//        }
        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

    }

    @Override
    public void stop() {
        cryptoboxDetector.disable();
    }
}