package com.qualcomm.hardware.Maxbotix;

/**
 * Created by Evan McLoughlin on 12/18/2017.
 */

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.HashMap;


@I2cSensor(name = "MaxSonar I2CXL v2", description = "MaxSonar I2CXL Sensor from MaxBotix", xmlTag = "MaxSonarI2CXLv2")
public class I2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Maxbotics;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "MaxSonarI2CXLv2";
    }

    public I2CXL(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected I2CXL(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public void setI2cAddress(I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    private void ping()
    {
        deviceClient.write8(0, 0x51, I2cWaitControl.WRITTEN);
    }

    public int getDistance()
    {
        int lastDistance = -1;
        long lastPingTime;
        long curTime;
        boolean waitingForNextPing = false;

        ping();
        lastPingTime = System.currentTimeMillis();

        while (!waitingForNextPing) {
            curTime = System.currentTimeMillis();
            if((curTime - lastPingTime) > 80){
                int potentialDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
                lastDistance = potentialDistance;
                waitingForNextPing = true;
            }
        }

        return lastDistance;

        /*
        if(((curTime - lastPingTime) > 80) && !waitingForNextPing)
        {
            int potentialDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));

            lastDistance = potentialDistance;

            waitingForNextPing = true;
        }
        if((System.currentTimeMillis() - lastPingTime) > 100)
        {
            ping();
            lastPingTime = System.currentTimeMillis();
            waitingForNextPing = false;
        }

        return lastDistance;
        */

    }

    public static int mode(int []array)
    {
        HashMap<Integer,Integer> hm = new HashMap<Integer,Integer>();
        int max  = 1;
        int temp = 0;

        for(int i = 0; i < array.length; i++) {

            if (hm.get(array[i]) != null) {

                int count = hm.get(array[i]);
                count++;
                hm.put(array[i], count);

                if(count > max) {
                    max  = count;
                    temp = array[i];
                }
            }

            else
                hm.put(array[i],1);
        }
        return temp;
    }

    public int sampleDistance(){

        //declare the array
        int distanceArray[];

        //allocate memory for 10 indices
        distanceArray = new int[10];

        for (int loopCount = 0; loopCount < distanceArray.length; loopCount++ ) {
            distanceArray[loopCount] = getDistance();
        }

        int distance = mode(distanceArray);

        return distance;


    }

    public int sampleDistance100(){

        //declare the array
        int distanceArray[];

        //allocate memory for 100 indices
        distanceArray = new int[100];

        for (int loopCount = 0; loopCount < distanceArray.length; loopCount++ ) {
            distanceArray[loopCount] = getDistance();
        }

        int distance = mode(distanceArray);

        return distance;


    }
}