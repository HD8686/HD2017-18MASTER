package org.firstinspires.ftc.hdcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.hdlib.General.Alliance;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDAuto;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;
import org.firstinspires.ftc.hdlib.StateMachines.HDStateMachine;

/**
 * Created by FIRSTMentor on 9/23/2017.
 */

public class Auto1 implements HDAuto {
    private HDDriveHandler robotDrive;
    private HDStateMachine SM;


    DcMotor frontLeft, frontRight, backLeft, backRight;
    AdafruitIMU IMU1;

    private enum States{
        delay,
    }

    public Auto1(double delay, Alliance alliance, HardwareMap hardwareMap){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        IMU1 = new AdafruitIMU("imu", 10);

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);

        SM = new HDStateMachine(robotDrive);

        robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robotDrive.resetEncoders();

        Runnable reset = new Runnable() {
            @Override
            public void run() {
                robotDrive.resetPIDvalues();
            }
        };

        SM.setResetCode(reset);
    }

    @Override
    public void start() {

    }

    @Override
    public void runLoop(double elapsedTime) {
        if(ready()){
            States states = (States) SM.getState();
            switch(states){
                case delay:
                    //Put delay code here
                    break;
            }
        }
    }

    private boolean ready(){
        boolean ready = true;
        if(!IMU1.isCalibrated()){
            ready = false;
        }
        return ready;
    }

}
