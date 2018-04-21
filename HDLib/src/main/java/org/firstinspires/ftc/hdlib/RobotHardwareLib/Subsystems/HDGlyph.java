package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.RobotHardwareLib.Servo.HDVexMotor;

/**
 * Created by FIRSTMentor on 3/24/2018.
 */

public class HDGlyph {

    private DcMotor leftIntake, rightIntake, liftMotor;
    private Servo leftBoxServo, rightBoxServo, glyphGripper, glyphStopper;
    private HDVexMotor glyphConveyor;

    public HDGlyph(DcMotor leftIntake, DcMotor rightIntake, DcMotor liftMotor, HDVexMotor glyphConveyor, Servo leftBoxServo, Servo rightBoxServo, Servo glyphGripper, Servo glyphStopper){
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;

        this.leftBoxServo = leftBoxServo;
        this.rightBoxServo = rightBoxServo;

        this.glyphConveyor = glyphConveyor;
        this.glyphGripper = glyphGripper;
        this.glyphStopper = glyphStopper;

        this.liftMotor = liftMotor;

        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stowBox();
        lowerGlyphGate();
        openBox();
    }

    public void lowerGlyphGate(){
        glyphStopper.setPosition(1);
    }

    public void raiseLiftGate(){
        glyphStopper.setPosition(0);
    }

    public void setIntakePower(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void setLiftPower(double power){
        liftMotor.setPower(power);
    }

    public void startConveyor(){
        glyphConveyor.setPower(-0.45);
    }

    public void stopConveyor(){
        glyphConveyor.setPower(0.0);
    }

    public void stowBox(){
        leftBoxServo.setPosition(.87);
        rightBoxServo.setPosition(.13);
    }

    public void extendBox(){
        leftBoxServo.setPosition(.27);
        rightBoxServo.setPosition(.73);
    }

    public void flatBox(){
        leftBoxServo.setPosition(.72);
        rightBoxServo.setPosition(.28);
    }

    public void gripBox(){
        glyphGripper.setPosition(.89);
    }

    public void openBox(){
        glyphGripper.setPosition(.57);
    }


}
