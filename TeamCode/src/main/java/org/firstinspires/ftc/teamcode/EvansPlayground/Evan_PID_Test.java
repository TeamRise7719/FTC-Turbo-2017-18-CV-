package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.hardware.Maxbotix.I2CXL;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
 * Created by Evan on 12/5/2017.
 */

@Autonomous
public class Evan_PID_Test extends LinearOpMode {

    DcMotor lb, lf, rb, rf;

    Telemetry telemetry;

    I2CXL ultrasonicFront;
    I2CXL ultrasonicBack;
    I2CXL ultrasonicRight;
    I2CXL ultrasonicLeft;

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

    @Override
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
        ultrasonicFront = hardwareMap.get(I2CXL.class, "ultsonFront");
        ultrasonicBack = hardwareMap.get(I2CXL.class, "ultsonBack");
        ultrasonicLeft = hardwareMap.get(I2CXL.class, "ultsonLeft");
        ultrasonicRight = hardwareMap.get(I2CXL.class, "ultsonRight");

        ultrasonicFront.initialize();
        ultrasonicBack.initialize();
        ultrasonicLeft.initialize();
        ultrasonicRight.initialize();

        lb = hardwareMap.dcMotor.get("1");
        lf = hardwareMap.dcMotor.get("2");
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb = hardwareMap.dcMotor.get("3");
        rf = hardwareMap.dcMotor.get("4");
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //-----------------------------------------=+(Hardware Map)+=-----------------------------------------\\

        //-------------------------------------=+(Initialization Config)+=------------------------------------\\
        srvo.raiseJewel();
        telemetry.addData(">", "Robot Ready!");
        telemetry.update();
        waitForStart();
        //-------------------------------------=+(Initialization Config)+=------------------------------------\\


        while (opModeIsActive()){

            while (opModeIsActive() && ultrasonicFront.sampleDistance() > 25){
                lb.setPower(-0.3);
                lf.setPower(-0.3);
                rb.setPower(-0.3);
                rf.setPower(-0.3);
            }

            break;
        }

    }




}
