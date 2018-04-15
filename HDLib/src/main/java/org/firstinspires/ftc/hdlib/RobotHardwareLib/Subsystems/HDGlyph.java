package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FIRSTMentor on 3/24/2018.
 */

public class HDGlyph {

    private DcMotor leftIntake, rightIntake, liftMotor;
    private Servo leftBoxServo, rightBoxServo;

    public HDGlyph(DcMotor leftIntake, DcMotor rightIntake, DcMotor liftMotor, Servo leftBoxServo, Servo rightBoxServo){
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;

        this.leftBoxServo = leftBoxServo;
        this.rightBoxServo = rightBoxServo;

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
    }

    public void setIntakePower(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void setLiftPower(double power){
        liftMotor.setPower(power);
    }

    public void stowBox(){
        leftBoxServo.setPosition(.5);
        rightBoxServo.setPosition(.5);
    }




}
