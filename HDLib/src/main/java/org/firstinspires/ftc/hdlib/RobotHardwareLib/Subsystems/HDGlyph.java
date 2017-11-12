package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.RobotHardwareLib.Servo.HDVexMotor;

/**
 * Created by FIRSTMentor on 11/11/2017.
 */

public class HDGlyph {

    public DcMotor scotchYokeMotor, leftPinionMotor, rightPinionMotor;
    public HDVexMotor bottomLeftIntake, bottomRightIntake, topLeftIntake, topRightIntake;
    public Servo blockKicker;
    //public Servo leftBlockGrabber, rightBlockGrabber;

    public HDGlyph(HardwareMap hardwareMap){

        scotchYokeMotor = hardwareMap.dcMotor.get("scotchYokeMotor");
        leftPinionMotor = hardwareMap.dcMotor.get("leftPinionMotor");
        rightPinionMotor = hardwareMap.dcMotor.get("rightPinionMotor");

        bottomLeftIntake = new HDVexMotor(hardwareMap, "bottomLeftIntake", Servo.Direction.FORWARD);
        bottomRightIntake = new HDVexMotor(hardwareMap, "bottomRightIntake", Servo.Direction.FORWARD);
        topLeftIntake = new HDVexMotor(hardwareMap, "topLeftIntake", Servo.Direction.FORWARD);
        topRightIntake = new HDVexMotor(hardwareMap, "topRightIntake", Servo.Direction.FORWARD);

        blockKicker = hardwareMap.servo.get("blockKicker");
        //leftBlockGrabber = hardwareMap.servo.get("leftBlockGrabber");
        //rightBlockGrabber = hardwareMap.servo.get("rightBlockGrabber");

        rightPinionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPinionMotor.setPower(1);
        rightPinionMotor.setPower(1);
        leftPinionMotor.setTargetPosition(0);
        rightPinionMotor.setTargetPosition(0);
        scotchYokeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
