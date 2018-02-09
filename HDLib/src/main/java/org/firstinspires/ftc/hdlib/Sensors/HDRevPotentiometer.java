package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDLoopInterface;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by FIRSTMentor on 12/22/2017.
 */

public class HDRevPotentiometer
{

    AnalogInput potentiometer;

    public HDRevPotentiometer(HardwareMap hardwareMap, String sensorName){
        potentiometer = hardwareMap.analogInput.get(sensorName);
    }


    public double getVoltage(){
        return potentiometer.getVoltage();
    }
}
