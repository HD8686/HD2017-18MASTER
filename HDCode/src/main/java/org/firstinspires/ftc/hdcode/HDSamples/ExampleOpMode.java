package org.firstinspires.ftc.hdcode.HDSamples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;
import org.firstinspires.ftc.hdlib.StateMachines.HDStateMachine;
import org.firstinspires.ftc.hdlib.StateMachines.HDWaitTypes;


/**
 * Created by Akash on 5/7/2016.
 */


@Autonomous(name = "ExampleOpMode", group = "HDSamples")
public class ExampleOpMode extends HDOpMode {

    HDDriveHandler robotDrive;
    HDStateMachine SM;

    DcMotor frontLeft, frontRight, backLeft, backRight;
    AdafruitIMU IMU1;

    private enum exampleStates{
        delay,
        driveForward,
        wait1,
        gyroTurn,
        wait2,
        driveForward2,
        DONE,
    }

    @Override
    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        IMU1 = new AdafruitIMU("imu", 10);

        robotDrive = new HDDriveHandler(frontLeft, backLeft, frontRight, backRight, true, -180, 180);

        SM = new HDStateMachine(robotDrive);

        robotDrive.reverseSide(HDDriveHandler.Side.Left);
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
    public void initializeLoop() {

    }


    @Override
    public void Start() {
        SM.setState(exampleStates.delay);
    }

    @Override
    public void continuousRun(double elapsedTime) {
        if(ready()){
            exampleStates states = (exampleStates) SM.getState();
                switch (states){
                    case delay:
                        SM.setNextState(exampleStates.driveForward, HDWaitTypes.Timer, 2.5);
                        break;
                    case driveForward:
                        SM.setNextState(exampleStates.wait1, HDWaitTypes.EncoderChangeBoth, 2500);
                        robotDrive.tankDrive(.25, .25);
                        break;
                    case wait1:
                        SM.setNextState(exampleStates.gyroTurn, HDWaitTypes.Timer, .25);
                        robotDrive.motorBreak();
                        break;
                    case gyroTurn:
                        SM.setNextState(exampleStates.wait2, HDWaitTypes.driveHandlerTarget);
                        robotDrive.gyroTurn(90.0, 0.009, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, IMU1.getZheading());
                        break;
                    case wait2:
                        SM.setNextState(exampleStates.driveForward2, HDWaitTypes.Timer, .25);
                        robotDrive.motorBreak();
                        break;
                    case driveForward2:
                        SM.setNextState(exampleStates.DONE, HDWaitTypes.EncoderChangeBoth, 2500);
                        robotDrive.tankDrive(.25, .25);
                        break;
                    case DONE:
                        robotDrive.motorBreak();
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
