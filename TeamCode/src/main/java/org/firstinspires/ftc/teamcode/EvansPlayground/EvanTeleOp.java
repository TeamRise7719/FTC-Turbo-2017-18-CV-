package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Driving.FestusLift;
import org.firstinspires.ftc.teamcode.subsystems.Driving.ServoManagementV2;

/**
 * Created by Evan McLoughlin on 11/14/2017.
 */
@TeleOp(name = "EvanTeleOp", group = "Festus")
public class EvanTeleOp extends OpMode {

    private FestusDrivetrain robot;
    private FestusLift lift;
    ServoManagementV2 srvo;

    boolean left_bumperState= false;
    boolean right_bumperState= false;
    boolean rotationToggleState = false;

    private boolean isReady = false;

    @Override
    public void init() {
        //Initialize robot
        robot = new FestusDrivetrain(hardwareMap, telemetry);
        robot.runUsingEncoders();
        lift = new FestusLift(hardwareMap, telemetry);
        lift.init();

        //Initialize Servos
        srvo = new ServoManagementV2(hardwareMap);
        srvo.init();

        isReady = true;
    }

    @Override
    public void init_loop() {
        if(isReady==true) {
            telemetry.addData(">", "Press X to Reset Glyph Encoders");
            telemetry.addData("Glyph Reset?", lift.glyphReset);
            telemetry.addData(">", "Robot Ready!");
            telemetry.update();

            if (gamepad1.x) {
                lift.resetGlyphRotateMotor();
            }
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        srvo.knockJewel(0);

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\
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
        if ((gamepad2.left_bumper)&&(!left_bumperState)) {
            if(lift.glyphRotated){
                srvo.toggleClaw1();
            }
            else {
                srvo.toggleClaw2();
            }
        }
        left_bumperState = gamepad2.left_bumper;

        if ((gamepad2.right_bumper)&&(!right_bumperState)) {
            if(lift.glyphRotated){
                srvo.toggleClaw2();
            }
            else {
                srvo.toggleClaw1();
            }
        }
        right_bumperState = gamepad2.right_bumper;


        if ((gamepad2.left_stick_y < -.5)&&(!gamepad2.x)) {
            if(lift.glyphRotated){
                srvo.clawIntake1();
            }
            else {
                srvo.clawIntake2();
            }
        } else if ((gamepad2.left_stick_y > .5)&&(!gamepad2.x)) {
            if(lift.glyphRotated){
                srvo.clawEject1();
            }
            else {
                srvo.clawEject2();
            }
        } else {
            if(lift.glyphRotated){
                srvo.clawStop1();
            }
            else {
                srvo.clawStop2();
            }
        }

        if ((gamepad2.right_stick_y < -.5)&&(!gamepad2.x)) {
            if(lift.glyphRotated){
                srvo.clawIntake2();
            }
            else {
                srvo.clawIntake1();
            }
        } else if ((gamepad2.right_stick_y > .5)&&(!gamepad2.x)) {
            if(lift.glyphRotated){
                srvo.clawEject2();
            }
            else {
                srvo.clawEject1();
            }
        } else {
            if(lift.glyphRotated){
                srvo.clawStop2();
            }
            else {
                srvo.clawStop1();
            }
        }
        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\


        //----------------------------------------------=+(Relic)+=----------------------------------------------\\
        if (gamepad1.right_bumper) {
            srvo.openRelic();
        } else if (gamepad1.left_bumper) {
            srvo.closeRelic();
        }
        if (gamepad1.b) {
            srvo.relicRelease();
        }

        if (gamepad2.dpad_up) {
            srvo.rotateUp();
        } else if (gamepad2.dpad_down) {
            srvo.rotateDown();
        }

        if((gamepad1.left_trigger>0.1)&&(Math.abs(gamepad1.right_trigger)<0.1)) {
            robot.winch(-gamepad1.left_trigger);
        }
        else if((gamepad1.right_trigger>0.1)&&(Math.abs(gamepad1.left_trigger)<0.1)) {
            robot.winch(gamepad1.right_trigger);
        }
        else{
            robot.winch(0);
        }
        //----------------------------------------------=+(Relic)+=----------------------------------------------\\

        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

        if (gamepad2.y) {
            lift.setLiftPower(-1);
        } else if (gamepad2.a) {
            lift.setLiftPower(1);
        } else {
            lift.setLiftPower(0);
        }

        if((gamepad2.x)&&(!gamepad2.left_bumper)&&(!gamepad2.right_bumper)&&(!rotationToggleState)){
            lift.rotateGlyph();
        }
        rotationToggleState = gamepad2.x;

        lift.glyphOverride(gamepad2.b);
        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

    }

    @Override
    public void stop() {
        super.stop();
        srvo.clawStop1();
        srvo.clawStop2();
    }
}