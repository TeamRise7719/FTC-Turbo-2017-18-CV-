package org.firstinspires.ftc.teamcode.DiagnosticOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ServoManagementV2;

/**
 * Created by Evan on 11/6/17.
 */
@TeleOp(name = "ServoTest", group = "Festus")
public class Servo_Test extends OpMode {
    //Import 2017 Drivetrain Subsystems
    Drivetrain drivetrain;
    ServoManagementV2 srvo;
    DcMotor liftMotor;





public void init() {

//    public void init() {

    //Initialize Drivetrain
    drivetrain = new Drivetrain(hardwareMap);

    //Initialize Servos
    srvo = new ServoManagementV2(hardwareMap);
    srvo.init();

    //Initialize lift and lift encoders
    liftMotor = hardwareMap.dcMotor.get("lift");
    liftMotor.setDirection(DcMotor.Direction.REVERSE);
    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    telemetry.addData("MAKE SURE THAT THE LIFT ", "IS IN STARTING POSITION");

}

    public void loop() {
        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\

        drivetrain.drive(gamepad1);

        //----------------------------------------------=+(Drivetrain)+=----------------------------------------------\\


        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\
        //Jewels
        if(gamepad2.dpad_down){
            srvo.lowerJewel();
        }
        else if(gamepad2.dpad_up){
            srvo.raiseJewel();
        }
        //----------------------------------------------=+(Jewel Arm)+=-----------------------------------------------\\




        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\
        if (gamepad2.x) {
            srvo.openClaw();
        } else if (gamepad2.b) {
            srvo.closeClaw();
        }
        //srvo.claw(gamepad2);
        //----------------------------------------------=+(Glyph Claw)+=----------------------------------------------\\




        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

        if (gamepad2.y) {
            drivetrain.raiseLift(.8);
        } else if (gamepad2.a) {
            drivetrain.lowerLift(.8);
        } else {
            drivetrain.stopLift();
        }

        telemetry.addData("encoders", liftMotor.getCurrentPosition());
        telemetry.update();



        if (liftMotor.getCurrentPosition() < -100 || liftMotor.getCurrentPosition() > 100) {

            if (gamepad2.left_bumper) {

                while (liftMotor.getCurrentPosition() < -100 || liftMotor.getCurrentPosition() > 100) {

                    if (liftMotor.getCurrentPosition() < -100){
                        drivetrain.raiseLift(.8);
                    }

                    else if (liftMotor.getCurrentPosition() > 100) {
                        drivetrain.lowerLift(.8);

                }
            }
        }
    }
        //----------------------------------------------=+(Glyph Lift)+=----------------------------------------------\\

    }
}
