package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

        setRelicBeakClosed();

    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void setRelicArmUp(){
        relicArm.setPosition(0.8);
    }

    public void setRelicArmDown(double adjustment){
        relicArm.setPosition(Range.clip(0.025 + adjustment, 0, 1));
    }

    public void setRelicBeakClosed(){
        relicBeak.setPosition(1.0);
    }

    public void setRelicBeakOpen(){
       relicBeak.setPosition(0.0);
    }

}
