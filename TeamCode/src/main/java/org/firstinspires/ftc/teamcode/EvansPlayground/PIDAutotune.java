package org.firstinspires.ftc.teamcode.EvansPlayground;

import android.provider.Settings;

/**
 * Created by Evan on 1/29/2018.
 */

public class PIDAutotune {
    boolean isMax, isMin;
    double input, output;
    double setpoint;
    double noiseBand;
    int controlType;
    boolean running;
    long peak1, peak2, lastTime;
    int sampleTime;
    int nLookBack;
    int peakType;
    double[] lastInputs = new double[101];
    double[] peaks = new double[10];
    int peakCount;
    boolean justchanged;
    boolean justevaled;
    double absMax, absMin;
    double oStep;
    double outputStart;
    double Ku, Pu;

    public PIDAutotune() {
        input = 0;
        output = 0;
        controlType =0 ; //default to PI
        noiseBand = 0.5;
        running = false;
        oStep = 30;
        SetLookbackSec(10);
        lastTime = System.currentTimeMillis();
    }

    public int Runtime(double Input){
        justevaled=false;
        if(peakCount>9 && running)
        {
            running = false;
            FinishUp();
            return 1;
        }
        long now = System.currentTimeMillis();

        if((now-lastTime)<sampleTime){
            return 0;
        }

        lastTime = now;
        double refVal = Input;
        justevaled=true;
        if(!running)
        { //initialize working variables the first time around
            peakType = 0;
            peakCount=0;
            justchanged=false;
            absMax= refVal;
            absMin= refVal;
            setpoint = refVal;
            running = true;
            outputStart = output;
		    output = outputStart+oStep;
        }
        else
        {
            if(refVal>absMax) absMax=refVal;
            if(refVal<absMin) absMin=refVal;
        }

        //oscillate the output base on the input's relation to the setpoint

        if(refVal>setpoint+noiseBand){
            output = outputStart-oStep;
        }

	    else if (refVal<setpoint-noiseBand){
            output = outputStart+oStep;
        }


        //bool isMax=true, isMin=true;
        isMax=true;isMin=true;
        //id peaks
        for(int i=nLookBack-1;i>=0;i--)
        {
            double val = lastInputs[i];
            if(isMax) isMax = refVal>val;
            if(isMin) isMin = refVal<val;
            lastInputs[i+1] = lastInputs[i];
        }
        lastInputs[0] = refVal;
        if(nLookBack<9)
        {  //we don't want to trust the maxes or mins until the inputs array has been filled
            return 0;
        }

        if(isMax)
        {
            if(peakType==0)peakType=1;
            if(peakType==-1)
            {
                peakType = 1;
                justchanged=true;
                peak2 = peak1;
            }
            peak1 = now;
            peaks[peakCount] = refVal;

        }
        else if(isMin)
        {
            if(peakType==0)peakType=-1;
            if(peakType==1)
            {
                peakType=-1;
                peakCount++;
                justchanged=true;
            }

            if(peakCount<10)peaks[peakCount] = refVal;
        }

        if(justchanged && peakCount>2)
        { //we've transitioned.  check if we can autotune based on the last peaks
            double avgSeparation = (Math.abs(peaks[peakCount-1]-peaks[peakCount-2])+Math.abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;
            if( avgSeparation < 0.05*(absMax-absMin))
            {
                FinishUp();
                running = false;
                return 1;

            }
        }
        justchanged=false;
        return 0;
    }
    // * Similar to the PID Compute function, returns 0 when done
    
    public void SetOutputStep(double Step){
        oStep = Step;
    }
    // * how far above and below the starting value will the output step?

    public double GetOutputStep(){
        return oStep;
    }
    //

    public void SetControlType(int Type){
        controlType = Type;
    }
    // * Determies if the tuning parameters returned will be PI (D=0)

    public int GetControlType(){
        return controlType;
    }//   or PID.  (0=PI, 1=PID)

    public void SetLookbackSec(int value){
        if (value<1) value = 1;

        if(value<25)
        {
            nLookBack = value * 4;
            sampleTime = 250;
        }
        else
        {
            nLookBack = 100;
            sampleTime = value*10;
        }
    }
    // * how far back are we looking to identify peaks

    public int GetLookbackSec(){
        return nLookBack * sampleTime / 1000;
    }//

    public void SetNoiseBand(double Band){
        noiseBand = Band;
    }
    // * the autotune will ignore signal chatter smaller than this value

    public double GetNoiseBand(){
        return noiseBand;
    }
    //   this should be acurately set

    public double GetKp(){
        return controlType==1 ? 0.6 * Ku : 0.4 * Ku;
    }
    // * once autotune is complete, these functions contain the

    public double GetKi(){
        return controlType==1? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
    }
    //   computed tuning parameters.

    public double GetKd(){
        return controlType==1? 0.075 * Ku * Pu : 0;
    }
    //Kd = Kc * Td

    public double GetOutput(){
        return output;
    }
    private void FinishUp() {
        output = outputStart;
        //we can generate tuning parameters!
        Ku = 4 * (2 * oStep) / ((absMax - absMin) * 3.14159);
        Pu = (double) (peak1 - peak2) / 1000;
    }
}
