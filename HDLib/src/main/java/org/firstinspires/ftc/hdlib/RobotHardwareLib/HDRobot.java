package org.firstinspires.ftc.hdlib.RobotHardwareLib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Servo.HDVexMotor;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDGlyph;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDJewel;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDRelic;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;
import org.firstinspires.ftc.hdlib.Sensors.HDMaxbotixUS;

/**
 * Created by FIRSTMentor on 11/4/2017.
 */

public class HDRobot {

    public HDDriveHandler robotDrive;
    public HDJewel robotJewel;
    public HDGlyph robotGlyph;
    public HDRelic robotRelic;
    public DcMotor frontLeft, frontRight, backLeft, backRight, leftIntake, rightIntake, liftMotor, relicLiftMotor;
    public AdafruitIMU IMU1;
    public HDVexMotor glyphConveyor;
    public ModernRoboticsI2cRangeSensor backUS, frontUS, rightUS, leftUS;
    public Servo leftBoxServo, rightBoxServo, glyphStopper, glyphGripper, relicBeak, relicArm;
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
        relicLiftMotor = hardwareMap.dcMotor.get("relicLiftMotor");

        glyphConveyor = new HDVexMotor(hardwareMap, "glyphConveyer", Servo.Direction.FORWARD);
        glyphGripper = hardwareMap.servo.get("glyphGripper");
        glyphStopper = hardwareMap.servo.get("glyphStopper");
        leftBoxServo = hardwareMap.servo.get("leftBoxTilt");
        rightBoxServo = hardwareMap.servo.get("rightBoxTilt");
        relicBeak = hardwareMap.servo.get("relicBeak");
        relicArm = hardwareMap.servo.get("relicArm");

        liftTouch = hardwareMap.get(DigitalChannel.class, "liftTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);

        frontUS = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "US1");
        backUS = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "US2");
        rightUS = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "US3");
        leftUS = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "US4");

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);
        robotGlyph = new HDGlyph(leftIntake, rightIntake, liftMotor, glyphConveyor, leftBoxServo, rightBoxServo, glyphGripper, glyphStopper);
        robotRelic = new HDRelic(relicLiftMotor,relicBeak, relicArm);
        robotJewel = new HDJewel(hardwareMap);
    }

}
