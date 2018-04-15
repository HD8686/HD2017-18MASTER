package org.firstinspires.ftc.hdlib.RobotHardwareLib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDGlyph;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDJewel;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;
import org.firstinspires.ftc.hdlib.Sensors.HDMaxbotixUS;

/**
 * Created by FIRSTMentor on 11/4/2017.
 */

public class HDRobot {

    public HDDriveHandler robotDrive;
    public HDJewel robotJewel;
    public HDGlyph robotGlyph;
    public DcMotor frontLeft, frontRight, backLeft, backRight, leftIntake, rightIntake, liftMotor;
    public AdafruitIMU IMU1;
    ModernRoboticsI2cRangeSensor backUS, frontUS, rightUS, leftUS;
    public Servo leftBoxServo, rightBoxServo;
    public DigitalChannel liftTouch;

    public HDRobot(HardwareMap hardwareMap){
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro currently calibrating...");
        IMU1 = new AdafruitIMU("imu", 50);
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro calibration complete!");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        leftBoxServo = hardwareMap.servo.get("leftBoxTilt");
        rightBoxServo = hardwareMap.servo.get("rightBoxTilt");
        liftTouch = hardwareMap.get(DigitalChannel.class, "liftTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);

        backUS = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "US1");

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);
        robotGlyph = new HDGlyph(leftIntake, rightIntake, liftMotor, leftBoxServo, rightBoxServo);
        robotJewel = new HDJewel(hardwareMap);
    }

}
