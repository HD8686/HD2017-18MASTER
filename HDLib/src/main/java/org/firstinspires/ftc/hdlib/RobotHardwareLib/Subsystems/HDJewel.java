package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by FIRSTMentor on 3/31/2018.
 */

public class HDJewel {

    public enum jewelColor{
        RED,
        BLUE,
        INCONCLUSIVE
    }

    public Servo jewelTiltServo;
    public Servo jewelHitServo;
    private ColorSensor jewelColorSensor;
    private DistanceSensor jewelDistance;

    private boolean jewelArmEnabled = true;

    private double jewelTiltStowed = 0.39;
    private double jewelTiltHit = 0.785;

    private double hitFront = 0.56; //flipped these due to color sensor on the wrong side
    private double hitBack = .32;
    private double perpendicularStowed = 0.45; //.42

    public HDJewel(HardwareMap hardwareMap){


        jewelDistance = hardwareMap.get(DistanceSensor.class, "jewelColor");
        jewelColorSensor = hardwareMap.get(ColorSensor.class, "jewelColor");

        jewelTiltServo = hardwareMap.servo.get("jewelTilt");
        jewelHitServo = hardwareMap.servo.get("jewelHit");

        raiseArm();
        stowPerpendicular();


        jewelColorSensor.enableLed(true);
    }

    public void resetJewel(){
        raiseArm();
        stowPerpendicular();
    }

    public void setJewelArmDisabled(){
        jewelArmEnabled = false;
    }

    public void setJewelArmEnabled(){
        jewelArmEnabled=true;
    }

    public void hitBack(){
        if(jewelArmEnabled)
            jewelHitServo.setPosition(hitFront);
    }

    public void hitFront(){
        if(jewelArmEnabled)
            jewelHitServo.setPosition(hitBack);
    }

    public void stowPerpendicular(){
            jewelHitServo.setPosition(perpendicularStowed);
    }

    public void lowerArm(){
        if(jewelArmEnabled) {
            jewelHitServo.setPosition(.42);
            jewelTiltServo.setPosition(jewelTiltHit);
        }
    }

    public void raiseArm(){
        jewelTiltServo.setPosition(jewelTiltStowed);
    }

    public jewelColor getLeftColor(){
        int redValue = jewelColorSensor.red();
        int greenValue = jewelColorSensor.green();
        int blueValue = jewelColorSensor.blue();
        double colorDistance = jewelDistance.getDistance(DistanceUnit.CM);
        boolean isBall = (colorDistance < 200 && colorDistance > 0);
        boolean isRed = redValue > blueValue && redValue > greenValue;
        boolean isBlue = blueValue > redValue && blueValue > greenValue;
        if(isRed && isBlue){
            return jewelColor.INCONCLUSIVE;
        } else if(isRed && isBall){
            return jewelColor.BLUE;
        } else if(isBlue & isBall){
            return jewelColor.RED;
        } else{
            return jewelColor.INCONCLUSIVE;
        }
    }
}
