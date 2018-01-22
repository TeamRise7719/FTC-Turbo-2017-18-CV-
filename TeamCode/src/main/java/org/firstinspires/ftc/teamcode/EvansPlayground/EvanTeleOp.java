package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;

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
        robot.runUsingEncoders();

        //Initialize Servos
        srvo = new ServoManagementV2(hardwareMap);

    }

    @Override
    public void start() {
        super.start();
        srvo.init();
    }

    @Override
    public void loop() {
        srvo.knockJewel(0);

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
        robot.arcadeMode = true;
        robot.drive(gamepad1, telemetry);

        if (gamepad1.x) {
            robot.resetHeading();
        }

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\
        //Keep Jewel Arm Up
        srvo.raiseJewel();
        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\


        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\
        if ((gamepad2.left_bumper)&&(!gamepad2.x)) {
            if(robot.glyphRotated){
                srvo.toggleClaw1();
                srvo.enableClaw1 = false;
            }
            else {
                srvo.toggleClaw2();
                srvo.enableClaw2 = false;
            }
        }
        else if (!gamepad2.left_bumper) {
            if(robot.glyphRotated){
                srvo.enableClaw1 = true;
            }
            else {
                srvo.enableClaw2 = true;
            }
        }

        if ((gamepad2.right_bumper)&&(!gamepad2.x)) {
            if(robot.glyphRotated){
                srvo.toggleClaw2();
                srvo.enableClaw2 = false;
            }
            else {
                srvo.toggleClaw1();
                srvo.enableClaw1 = false;
            }
        }
        else if (!gamepad2.right_bumper) {
            if(robot.glyphRotated){
                srvo.enableClaw2 = true;
            }
            else {
                srvo.enableClaw1 = true;
            }
        }

        if (gamepad2.right_stick_y < -.5) {
            srvo.clawIntake();
        } else if (gamepad2.right_stick_y > .5) {
            srvo.clawEject();
        } else {
            srvo.clawStop();
        }

        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\


        //----------------------------------------------=+(Relic)+=----------------------------------------------\\
        if (gamepad1.right_bumper) {
            srvo.openRelic();
        } else if (gamepad1.left_bumper) {
            srvo.closeRelic();
        }
        if (gamepad1.left_trigger > 0.1) {
            srvo.relicRelease();
        }

        if (gamepad2.dpad_up) {
            srvo.rotateUp();
        } else if (gamepad2.dpad_down) {
            srvo.rotateDown();
        }

        robot.winch(gamepad2.left_stick_x);
        //----------------------------------------------=+(Relic)+=----------------------------------------------\\

        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

        if (gamepad2.y) {
            robot.raiseLift(1);
        } else if (gamepad2.a) {
            robot.lowerLift(1);
        } else {
            robot.stopLift();
        }

        if((gamepad2.x)&&(!gamepad2.left_bumper)&&(!gamepad2.right_bumper)){
            robot.rotateGlyph();
            robot.enableRotation = false;
        }
        else if (!gamepad2.x){
            robot.enableRotation = true;
        }

        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

    }

    @Override
    public void stop() {
        super.stop();
        srvo.clawStop();
    }
}