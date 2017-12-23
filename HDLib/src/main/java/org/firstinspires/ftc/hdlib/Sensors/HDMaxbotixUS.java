package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FIRSTMentor on 12/22/2017.
 */

public class HDMaxbotixUS
{
    double distanceConst = 0.004883;
    AnalogInput ultrasonic;

    public HDMaxbotixUS(HardwareMap hardwareMap, String sensorName){
        ultrasonic = hardwareMap.analogInput.get(sensorName);
    }

    public double getDistanceCM(){
        return ultrasonic.getVoltage()/distanceConst;
    }

}
