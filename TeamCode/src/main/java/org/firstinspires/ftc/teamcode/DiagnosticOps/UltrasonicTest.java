package org.firstinspires.ftc.teamcode.DiagnosticOps;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Sensing.Wire;

/**
 * Created by Admin on 12/17/2017.
 */
@TeleOp(name = "UltrasonicTest", group = "Festus")
public class UltrasonicTest extends OpMode {
    private Wire ds;
    private int readCount = 0;
    private int distance;
    private long pingTime;

    @Override
    public void init() {
        ds = new Wire(hardwareMap,"ultrasonic",0xE0);
        start();
    }

    public void start(){
        ds.beginWrite(0x51);
        ds.write(0);
        ds.endWrite();
        pingTime = System.currentTimeMillis();
    }

    @Override
    public void loop(){

        //Write to Register to get Data from Ultrasonic
        if((System.currentTimeMillis() -pingTime) > 100 ){
            ds.requestFrom(0,2);
            ds.beginWrite(0x51);
            ds.write(0);
            ds.endWrite();
            pingTime = System.currentTimeMillis();
        }

        //If Response is in Wire Library
        if(ds.responseCount() > 0){
            ds.getResponse();
            if(ds.isRead()){
                //Get Time From Wire Library
                long micros = ds.micros();
                //Get Distance From Wire Library
                distance = ds.readHL();
                //Not sure why this is 760 you may want to check this
                if(distance < 760) {
                    readCount++;
                    telemetry.addData("Count", readCount);
                    telemetry.addData("Time", micros/1000);
                    telemetry.addData("cm", distance);
                    telemetry.update();
                }
            }
        }
    }

    public void stop(){
        ds.close();
    }
}
