package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by baseball0206 on 4/22/2018.
 */

public class HDRelic {

    DcMotor liftMotor;
    Servo relicBeak;
    Servo relicArm;

    public HDRelic(DcMotor liftMotor, Servo relicBeak, Servo relicArm){

        this.liftMotor = liftMotor;
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.relicBeak = relicBeak;

        this.relicArm = relicArm;

    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void setRelicArmUp(){

    }

    public void setRelicArmDown(double position){
        //relicArm.setPosition(position);
    }

    public void setRelicBeakClosed(){
        //relicBeak.setPosition(.5);
    }

    public void setRelicBeakOpen(){
       // relicBeak.setPosition(0.0);
    }

}
