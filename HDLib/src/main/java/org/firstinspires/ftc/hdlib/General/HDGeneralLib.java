package org.firstinspires.ftc.hdlib.General;

/**
 * Created by Akash on 5/15/2016.
 * This class houses the general methods and functions used in most classes, which doesn't really belong in a specific class due to its wide usage.
 */

public class HDGeneralLib {
    public static double getCurrentTimeSeconds(){
        return System.currentTimeMillis()/1000.0;
    }

    public static boolean isDifferenceWithin(double x1, double x2, double difference){
        return (Math.abs(x1-x2) <= difference);

    }
}
