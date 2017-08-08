package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;


/**
 * Created by Akash on 8/16/2016.
 */
public class HDMRRange {
    private String rangeHMKey;
    private ModernRoboticsI2cRangeSensor rangeSensor;

    public HDMRRange(String rangeHMkey){
        rangeSensor = HDOpMode.getInstance().hardwareMap.get(ModernRoboticsI2cRangeSensor.class, rangeHMkey);
        this.rangeHMKey = rangeHMkey;
    }

    public double getUSValue(){
        return rangeSensor.rawUltrasonic();
    }

    public double getODSValue(){
        return rangeSensor.rawOptical();
    }

    public String getName(){
        return rangeHMKey;
    }

    public double getODSValueCM(){
        return rangeSensor.cmOptical();
    }

}