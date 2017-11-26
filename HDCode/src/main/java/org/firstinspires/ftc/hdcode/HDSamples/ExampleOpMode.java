package org.firstinspires.ftc.hdcode.HDSamples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.hdlib.General.Alliance;
import org.firstinspires.ftc.hdlib.OpModeManagement.AutoTransitioner;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.AdafruitIMU;
import org.firstinspires.ftc.hdlib.StateMachines.HDStateMachine;
import org.firstinspires.ftc.hdlib.StateMachines.HDWaitTypes;


/**
 * Created by Akash on 5/7/2016.
 */


//@Autonomous(name = "ExampleOpMode", group = "HDSamples")
public class ExampleOpMode extends HDOpMode {

    HDRobot robot;
    HDStateMachine SM;

    DcMotor frontLeft, frontRight, backLeft, backRight;

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
        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setAlliance(Alliance.RED_ALLIANCE);

        Runnable reset = new Runnable() {
            @Override
            public void run() {
                robot.robotDrive.resetPIDvalues();
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
                        SM.setNextState(exampleStates.wait1, HDWaitTypes.EncoderChangeBoth, 2500.0);
                        robot.robotDrive.tankDrive(.25, .25);
                        break;
                    case wait1:
                        SM.setNextState(exampleStates.gyroTurn, HDWaitTypes.Timer, 0.25);
                        robot.robotDrive.motorBreak();
                        break;
                    case gyroTurn:
                        SM.setNextState(exampleStates.wait2, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(90.0, 0.007, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                        break;
                    case wait2:
                        SM.setNextState(exampleStates.driveForward2, HDWaitTypes.Timer, .25);
                        robot.robotDrive.motorBreak();
                        break;
                    case driveForward2:
                        SM.setNextState(exampleStates.DONE, HDWaitTypes.EncoderChangeBoth, 2500.0);
                        robot.robotDrive.tankDrive(.25, .25);
                        break;
                    case DONE:
                        robot.robotDrive.motorBreak();
                        break;

                }


        }


    }

    private boolean ready(){
        boolean ready = true;
        if(!robot.IMU1.isCalibrated()){
            ready = false;
        }
        return ready;
    }




}
