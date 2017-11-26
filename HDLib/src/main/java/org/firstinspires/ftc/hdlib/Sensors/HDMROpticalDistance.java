package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;

/**
 * Created by Akash on 10/19/2016.
 */
public class HDMROpticalDistance {
    private String odsHMKey;
    private OpticalDistanceSensor ODS;

    public HDMROpticalDistance(String rangeHMkey){
        ODS = HDOpMode.getInstance().hardwareMap.opticalDistanceSensor.get(rangeHMkey);
        this.odsHMKey = rangeHMkey;
    }



    public String getName(){
        return odsHMKey;
    }

    public void enableLED(boolean enable){
        ODS.enableLed(enable);
    }

    public double getRawLightDetected(){
        return ODS.getRawLightDetected();
    }

    public double getLightDetected(){
        return ODS.getLightDetected();
    }

    public double getRawLightDetectedMax(){
        return ODS.getRawLightDetectedMax();
    }



}
