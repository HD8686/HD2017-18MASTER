package org.firstinspires.ftc.hdlib.RobotHardwareLib;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
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
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public AdafruitIMU IMU1;
    public HDJewel robotJewel;
    public HDGlyph robotGlyph;
    public HDMaxbotixUS backUS, frontUS;
    //public HDRelic robotRelic;
    private Servo tailHook;

    public HDRobot(HardwareMap hardwareMap){
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro currently calibrating...");
        IMU1 = new AdafruitIMU("imu", 50);
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro calibration complete!");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");
        tailHook = hardwareMap.servo.get("tailHook");

        //leftUS = new HDMaxbotixUS(hardwareMap, "leftUS");
        //rightUS = new HDMaxbotixUS(hardwareMap, "rightUS");
        backUS = new HDMaxbotixUS(hardwareMap, "backUS");
        frontUS = new HDMaxbotixUS(hardwareMap, "frontUS");

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight,tailHook, true, -180, 180);
        robotGlyph = new HDGlyph(hardwareMap);
        robotJewel = new HDJewel(hardwareMap);
        //robotRelic = new HDRelic(hardwareMap);
    }

}
