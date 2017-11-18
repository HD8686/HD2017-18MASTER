package org.firstinspires.ftc.hdlib.RobotHardwareLib;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDGlyph;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDJewel;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;

/**
 * Created by FIRSTMentor on 11/4/2017.
 */

public class HDRobot {

    public HDDriveHandler robotDrive;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public AdafruitIMU IMU1;
    public HDJewel robotJewel;
    public HDGlyph robotGlyph;
    public ColorSensor bottomLeftColor, bottomRightColor;

    public HDRobot(HardwareMap hardwareMap){
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro currently calibrating...");
        IMU1 = new AdafruitIMU("imu", 10);
        HDOpMode.getInstance().dashboard.addDiagnosticSpecificTelemetry(0, "Gyro calibration complete!");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");
        bottomLeftColor = hardwareMap.get(ColorSensor.class, "Left_Bottom_Color");
        bottomRightColor = hardwareMap.get(ColorSensor.class, "Right_Bottom_Color");

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);
        robotGlyph = new HDGlyph(hardwareMap);
        robotJewel = new HDJewel(hardwareMap);
    }

}
