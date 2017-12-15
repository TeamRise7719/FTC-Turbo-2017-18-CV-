package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;


/**
 * Created by Evan McLoughlin on 12/15/2017.
 */


@I2cSensor(name = "Maxbotics UltSonic Ranging 1242", description = "I2c compatible narrow beam ranging sensor from Maxbotics. High interference tolerance.", xmlTag = "Maxbotics-1242")
public class Maxbotics_UltSonic_1242 extends I2cDeviceSynchDevice<I2cDeviceSynch>{

    @Override
    public Manufacturer getManufacturer() {

        return Manufacturer.Maxbotics;

    }

    @Override
    protected synchronized boolean doInitialize(){

        return true;

    }

    @Override
    public String getDeviceName(){

        return "Maxbotics 1242 - 000 Ultrasonic Ranging Sensor";

    }

    public Maxbotics_UltSonic_1242(I2cDeviceSynch deviceClient){

        super(deviceClient, true);

        //Maybe don't need this?
        //this.deviceClient.setI2cAddress(112);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();

    }

    public enum Register {

        //InitRangeRead(0xE0);
        //WriteRangeRead


    }








}
