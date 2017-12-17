package org.firstinspires.ftc.teamcode.subsystems.Sensing;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ultrasonic {

    HardwareMap hardwareMap;

    private Wire ds;
    private int distance;
    private long pingTime;

    public Ultrasonic(HardwareMap hardwareMapRef) {
        hardwareMap = hardwareMapRef;
        ds = new Wire(hardwareMap,"ultrasonic",0xE0);
    }

    public void start(){
        ds.beginWrite(0x51);
        ds.write(0);
        ds.endWrite();
        pingTime = System.currentTimeMillis();
    }

    public void stop(){
        ds.close();
    }

    public int getDistance() {
        distance = -1;
        while (distance != -1) {
            //Write to Register to get Data from Ultrasonic
            if ((System.currentTimeMillis() - pingTime) > 100) {
                ds.requestFrom(0, 2);
                ds.beginWrite(0x51);
                ds.write(0);
                ds.endWrite();
                pingTime = System.currentTimeMillis();
            }
            //If Response is in Wire Library
            if (ds.responseCount() > 0) {
                ds.getResponse();
                if (ds.isRead()) {
                    //Get Distance From Wire Library
                    distance = ds.readHL();
                }
            }
        }
        return distance;
    }
}
