package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by FIRSTMentor on 11/4/2017.
 */

public class HDJewel {

    public enum jewelColor{
        RED,
        BLUE,
        INCONCLUSIVE
    }

    private Servo leftServo, rightServo;
    private ColorSensor leftColor, rightColor;
    public DistanceSensor leftDistance, rightDistance;

    private double leftJewelUp = 0.09;
    private double rightJewelUp = 0.69;
    private double leftJewelDown = 0.69;
    private double rightJewelDown = 0.34;

    public HDJewel(HardwareMap hardwareMap){

        leftServo = hardwareMap.servo.get("Left_Jewel_Servo");
        rightServo = hardwareMap.servo.get("Right_Jewel_Servo");

        leftColor = hardwareMap.get(ColorSensor.class, "Left_Jewel_Color");
        rightColor = hardwareMap.get(ColorSensor.class, "Right_Jewel_Color");

        leftDistance = hardwareMap.get(DistanceSensor.class, "Left_Jewel_Color");
        rightDistance = hardwareMap.get(DistanceSensor.class, "Right_Jewel_Color");

        leftColor.enableLed(true);
        rightColor.enableLed(true);
    }

    public jewelColor getLeftColor(){
        int redValue = leftColor.red();
        int greenValue = leftColor.green();
        int blueValue = leftColor.blue();
        double colorDistance = leftDistance.getDistance(DistanceUnit.CM);
        boolean isBall = (colorDistance < 200 && colorDistance > 0);
        boolean isRed = redValue > blueValue && redValue > greenValue;
        boolean isBlue = blueValue > redValue && blueValue > greenValue;
        if(isRed && isBlue){
            return jewelColor.INCONCLUSIVE;
        } else if(isRed && isBall){
            return jewelColor.RED;
        } else if(isBlue & isBall){
            return jewelColor.BLUE;
        } else{
            return jewelColor.INCONCLUSIVE;
        }
    }

    public jewelColor getRightColor(){
        int redValue = rightColor.red();
        int greenValue = rightColor.green();
        int blueValue = rightColor.blue();
        double colorDistance = rightDistance.getDistance(DistanceUnit.CM);
        boolean isBall = (colorDistance < 200 && colorDistance > 0);
        boolean isRed = redValue > blueValue && redValue > greenValue;
        boolean isBlue = blueValue > redValue && blueValue > greenValue;
        if(isRed && isBlue){
            return jewelColor.INCONCLUSIVE;
        } else if(isRed && isBall){
            return jewelColor.RED;
        } else if(isBlue && isBall){
            return jewelColor.BLUE;
        } else{
            return jewelColor.INCONCLUSIVE;
        }
    }

    public void lowerRightServo(){
        rightServo.setPosition(rightJewelDown);
    }

    public void lowerLeftServo(){
        leftServo.setPosition(leftJewelDown);
    }

    public void raiseRightServo(){
        rightServo.setPosition(rightJewelUp);
    }

    public void raiseLeftServo(){
        leftServo.setPosition(leftJewelUp);
    }

}
