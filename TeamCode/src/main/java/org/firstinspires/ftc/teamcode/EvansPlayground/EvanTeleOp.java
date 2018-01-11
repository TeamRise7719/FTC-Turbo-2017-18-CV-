package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Driving.PID_Library;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;
import com.qualcomm.hardware.Maxbotix.I2CXL;

/**
 * Created by Evan McLoughlin on 11/14/2017.
 */
@TeleOp(name = "EvanTeleOp", group = "Festus")
public class EvanTeleOp extends OpMode {


    private FestusDrivetrain robot;


    ServoManagementV2 srvo;



    @Override
    public void init() {
        //Initialize robot
        robot = new FestusDrivetrain(hardwareMap, telemetry);



//        robot.runUsingEncoders()


        //Initialize Servos
        srvo = new ServoManagementV2(hardwareMap);
        srvo.init();

//        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
//        telemetry.addData("Arcade Mode (a)", robot.arcadeMode ? "YES" : "no.");
//        telemetry.update();
    }

    @Override
    public void loop() {
        srvo.knockJewel(0);


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
        //Keep Jewel Arm Up
        srvo.raiseJewel();
        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\


        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\
        if (gamepad2.b) {
            srvo.openClaw();
        } else if (gamepad2.x) {
            srvo.closeClaw();
        } else if (gamepad2.right_trigger > 0.1) {
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

        if (gamepad2.left_trigger > 0.1) {
            srvo.relicRelease();
        }


        robot.winch(gamepad2.left_stick_x);
        //----------------------------------------------=+(Relic)+=----------------------------------------------\\



//        telemetry.addData("srvo",srvo.getRotate());


        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

        if (gamepad2.y) {
            robot.raiseLift(1);
        } else if (gamepad2.a) {
            robot.lowerLift(1);
        } else {
            robot.stopLift();
        }



        //This is a very useful function, it just needs to be massively recalibrated to work with the current lift

        //!!!!Possibly deprecated because of new lift design, least priority!!!!

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
}