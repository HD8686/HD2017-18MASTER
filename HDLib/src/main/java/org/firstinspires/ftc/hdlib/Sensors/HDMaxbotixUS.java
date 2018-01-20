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

public class HDMaxbotixUS implements HDLoopInterface.LoopTimer
{

    private ElapsedTime timer;

    private boolean active = true;


    double average = 0.0;
    double distanceConst = 0.004883;
    AnalogInput ultrasonic;
    List<Double> list = new ArrayList<>(Collections.nCopies(5,0.0));

    public HDMaxbotixUS(HardwareMap hardwareMap, String sensorName){
        ultrasonic = hardwareMap.analogInput.get(sensorName);
        timer = new ElapsedTime();

        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.ContinuousRun);
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.InitializeLoop);
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.Start);
    }

    public void setActive(boolean active){
        this.active = active;
    }

    public double getDistanceCM(){
        return average;
    }

    @Override
    public void InitializeLoopOp() {
        if(timer.milliseconds() > 10 && active) {
            double reading = ultrasonic.getVoltage() / distanceConst;

            list.remove(0);
            list.add(reading);

            average = 0.0;

            for(double i: list){
                average += i;
            }

            average = average/list.size();

            timer.reset();
        }
    }

    @Override
    public void continuousCallOp() {
        if(timer.milliseconds() > 10 && active) {
            double reading = ultrasonic.getVoltage() / distanceConst;

            list.remove(0);
            list.add(reading);

            average = 0.0;

            for(double i: list){
                average += i;
            }

            average = average/list.size();

            timer.reset();
        }
    }

    @Override
    public void StartOp() {

    }
}
