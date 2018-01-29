package org.firstinspires.ftc.teamcode.EvansPlayground;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.subsystems.Driving.SeansEncLibrary;

import java.util.Random;

/**
 * Created by seancardosi on 10/25/17.
 */
@TeleOp(name = "Autotuner Test", group = "Festus")
@Disabled
public class Autotuner extends LinearOpMode {

    SeansEncLibrary enc;
    PIDAutotune autotuner;

    byte ATuneModeRemember=2;
    double input=80, output=50, setpoint=180;
    double kp=2,ki=0.5,kd=2;

    double kpmodel=1.5, taup=100;
    double[] theta = new double[50];
    double outputStart=5;
    double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
    int aTuneLookBack=20;

    boolean tuning = true;
    long  modelTime, serialTime;

    public void runOpMode() throws InterruptedException {

        autotuner = new PIDAutotune();
        autotuner.SetControlType(1);

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        output=aTuneStartValue;
        autotuner.SetNoiseBand(aTuneNoise);
        autotuner.SetOutputStep(aTuneStep);
        autotuner.SetLookbackSec((int)aTuneLookBack);

        waitForStart();

        while (opModeIsActive()) {

            enc.gyro_angle = enc.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            input = enc.gyro_angle.firstAngle;

            if(tuning)
            {
                int val = (autotuner.Runtime(input));
                if (val!=0)
                {
                    tuning = false;
                }
                if(!tuning)
                { //we're done, set the tuning parameters
                    kp = autotuner.GetKp();
                    ki = autotuner.GetKi();
                    kd = autotuner.GetKd();
                    telemetry.addData("P", kp);
                    telemetry.addData("I", ki);
                    telemetry.addData("D", kd);
                    telemetry.update();
                }
            }

            //Output to Motor
            //analogWrite(0,output);
            telemetry.addData("Output", autotuner.GetOutput());
            telemetry.update();
        }
    }
}
