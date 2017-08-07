package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by akash on 8/4/2017.
 */

public class AdafruitIMU {

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private double lastChecked = 0;
    private int updateLatency;

    public AdafruitIMU(String hardwareMapID, int updateLatency){
        this.updateLatency = updateLatency;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = HDOpMode.getInstance().hardwareMap.get(BNO055IMU.class, hardwareMapID);
        imu.initialize(parameters);
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file)));
    }


    //This method resets the Z heading of the gyro but it takes a while to do it.
    public void initializeIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file)));
    }

    public boolean isCalibrated(){
        return imu.isGyroCalibrated() && imu.isAccelerometerCalibrated();
    }

    public String getCalibrationStatus(){
        return imu.getCalibrationStatus().toString();
    }

    public String getStatus(){
        return imu.getSystemStatus().toShortString();
    }

    public double getXheading(){
       if((System.currentTimeMillis() - lastChecked > updateLatency) || (lastChecked == 0)){
           lastChecked = System.currentTimeMillis();
           angles   = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           gravity  = imu.getGravity();
           return angles.thirdAngle;
       }else{
           return angles.thirdAngle;
       }
    }

    public double getYheading(){
        if((System.currentTimeMillis() - lastChecked > updateLatency) || (lastChecked == 0)){
            lastChecked = System.currentTimeMillis();
            angles   = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            return angles.secondAngle;
        }else{
            return angles.secondAngle;
        }
    }

    public double getZheading(){
        if((System.currentTimeMillis() - lastChecked > updateLatency) || (lastChecked == 0)){
            lastChecked = System.currentTimeMillis();
            angles   = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            return -angles.firstAngle;
        }else{
            return -angles.firstAngle;
        }
    }


}
