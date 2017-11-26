package org.firstinspires.ftc.hdcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.hdlib.General.Alliance;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDAuto;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDJewel;
import org.firstinspires.ftc.hdlib.Sensors.HDVuforiaVuMarks;
import org.firstinspires.ftc.hdlib.StateMachines.HDStateMachine;
import org.firstinspires.ftc.hdlib.StateMachines.HDWaitTypes;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by FIRSTMentor on 9/23/2017.
 */

public class Auto2 implements HDAuto {

    private HDRobot robot;
    private HDStateMachine SM;

    private double delay;
    private Alliance alliance;
    private boolean turnLeft;

    private enum States{
        delay,
        raiseLift,
        lowerJewelArm,
        readJewel,
        turn,
        turnBack,
        driveOffBoard,
        straightenUp,
        driveForward,
        done
    }

    public Auto2(double delay, Alliance alliance, HardwareMap hardwareMap){

        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.setAlliance(alliance);

        robot.robotJewel.raiseRightServo();
        robot.robotJewel.raiseLeftServo();

        this.delay = delay;
        this.alliance = alliance;

        Runnable reset = new Runnable() {
            @Override
            public void run() {
                robot.robotDrive.resetPIDvalues();
            }
        };

        SM.setResetCode(reset);
    }

    @Override
    public void start() {
        SM.setState(States.delay);
    }

    @Override
    public void runLoop(double elapsedTime) {
        if(ready()){
            States states = (States) SM.getState();
            switch(states){
                case delay:
                    SM.setNextState(States.raiseLift, HDWaitTypes.Timer, delay);
                    break;
                case raiseLift:
                    SM.setNextState(States.lowerJewelArm, HDWaitTypes.Timer, 1.5);
                    robot.robotGlyph.setLiftPower(1);
                    break;
                case lowerJewelArm:
                    robot.robotGlyph.setLiftPower(0);
                    if(alliance == Alliance.RED_ALLIANCE){
                        robot.robotJewel.lowerRightServo();
                        if(robot.robotJewel.getRightColor() != HDJewel.jewelColor.INCONCLUSIVE){
                            SM.setNextState(States.readJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }else{
                        robot.robotJewel.lowerLeftServo();
                        if(robot.robotJewel.getLeftColor() != HDJewel.jewelColor.INCONCLUSIVE){
                            SM.setNextState(States.readJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }
                    break;
                case readJewel:
                    if(alliance == Alliance.RED_ALLIANCE){
                        if(robot.robotJewel.getRightColor() == HDJewel.jewelColor.RED){
                            turnLeft = false;
                            SM.setNextState(States.turn, HDWaitTypes.Timer, 0.05);
                        }else if(robot.robotJewel.getRightColor() == HDJewel.jewelColor.BLUE){
                            turnLeft = true;
                            SM.setNextState(States.turn, HDWaitTypes.Timer, 0.05);
                        }
                    }else{
                        if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.RED){
                            turnLeft = true;
                            SM.setNextState(States.turn, HDWaitTypes.Timer, 0.05);
                        }else if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.BLUE){
                            turnLeft = false;
                            SM.setNextState(States.turn, HDWaitTypes.Timer, 0.05);
                        }
                    }
                    break;
                case turn:
                    if(turnLeft){
                        SM.setNextState(States.turnBack, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(-12, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.turnBack, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(12, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }
                    break;
                case turnBack:
                    SM.setNextState(States.driveOffBoard, HDWaitTypes.driveHandlerTarget);
                    robot.robotJewel.raiseRightServo();
                    robot.robotJewel.raiseLeftServo();
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveOffBoard:
                    SM.setNextState(States.straightenUp, HDWaitTypes.EncoderChangeBoth, 825.0);
                    robot.robotDrive.tankDrive(.25, .25);
                    break;
                case straightenUp:
                    SM.setNextState(States.driveForward, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(0, 0.025, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveForward:
                    SM.setNextState(States.done, HDWaitTypes.EncoderChangeBoth, 825.0);
                    robot.robotDrive.tankDrive(.25, .25);
                    break;
                case done:
                    robot.robotGlyph.setIntakePower(0.0);
                    robot.robotGlyph.blockKickerIn();
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
