package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by FIRSTMentor on 3/24/2018.
 */

public class HDGlyph {

    DcMotor leftIntake;
    DcMotor rightIntake;

    public HDGlyph(DcMotor leftIntake, DcMotor rightIntake){
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;

        this.rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntakePower(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }




}
