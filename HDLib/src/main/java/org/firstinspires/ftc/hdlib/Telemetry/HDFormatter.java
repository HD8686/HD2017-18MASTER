package org.firstinspires.ftc.hdlib.Telemetry;

import java.text.NumberFormat;

/**
 * Created by FIRSTMentor on 11/17/2017.
 */

public class HDFormatter {

    NumberFormat percentFormat;


    public HDFormatter(){
        percentFormat = NumberFormat.getPercentInstance();
        percentFormat.setMinimumFractionDigits(1);
    }

    public String formatPercent(double percentage){
        return percentFormat.format(percentage);
    }

    public String formatDecimals(double number, int roundAmount){
        return String.format("%." + String.valueOf(roundAmount) + "f", number);
    }

}
