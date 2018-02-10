package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.Sensors.HDRevPotentiometer;

/**
 * Created by FIRSTMentor on 2/8/2018.
 */

public class HDRelic {

    DcMotor relicMotor;
    CRServo liftTilt;
    Servo relicArm;
    Servo beak;
    HDRevPotentiometer potentiometer;


    private double beakClosed = 0.11;
    private double beakGrab = 0.21;
    private double beakOpen = 0.55;
    private double relicArmClosed = 0.01;
    private double relicArmOpen = 0.93;


    public HDRelic(HardwareMap hardwareMap){
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTilt = hardwareMap.crservo.get("liftTilt");
        relicArm = hardwareMap.servo.get("relicArm");
        beak = hardwareMap.servo.get("beak");
        potentiometer = new HDRevPotentiometer(hardwareMap, "revPot");
    }

    public double getPotentiometerVoltage(){
        return  potentiometer.getVoltage();
    }

    public void setRelicMotorPower(double power){
        relicMotor.setPower(power);
    }

    public void setRelicTiltPower(double power){
        liftTilt.setPower(power);
    }



}
