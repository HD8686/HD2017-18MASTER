package org.firstinspires.ftc.hdcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class Auto3 implements HDAuto {

    private HDRobot robot;
    private HDStateMachine SM;
    private HDVuforiaVuMarks vuforiaVuMarks;
    RelicRecoveryVuMark vuMark;
    ElapsedTime failsafeTimer;

    private double delay;
    private Alliance alliance;
    private boolean turnLeft;

    private enum States{
        delay,
        lowerJewelArm,
        readJewel,
        turn,
        turnBack,
        driveOffBoard,
        straightenUp,
        backIntoStone,
        scanVuMark,
        driveForward,
        squareToWall,
        goToColumn,
        deposit,
        backOut,
        done
    }

    public Auto3(double delay, Alliance alliance, HardwareMap hardwareMap){

        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        vuforiaVuMarks = new HDVuforiaVuMarks(hardwareMap);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.setAlliance(alliance);

        robot.robotJewel.raiseRightServo();
        robot.robotJewel.raiseLeftServo();

        this.delay = delay;
        this.alliance = alliance;

        failsafeTimer = new ElapsedTime();

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
                    SM.setNextState(States.lowerJewelArm, HDWaitTypes.Timer, delay);
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
                    SM.runOnce(new Runnable() {
                        @Override
                        public void run() {
                            vuforiaVuMarks.activateTracking();
                            vuforiaVuMarks.flash(true);
                        }
                    });
                    SM.setNextState(States.backIntoStone, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(0, 0.025, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case backIntoStone:
                    SM.setNextState(States.scanVuMark, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.tankDrive(-0.2, -0.2);
                    break;
                case scanVuMark:
                    SM.setNextState(States.driveForward, HDWaitTypes.Timer, 5.0);
                    robot.robotDrive.motorBreak();
                    vuMark = vuforiaVuMarks.scan();
                    if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                        SM.resetValues();
                        SM.setState(States.driveForward);
                    }
                    break;
                case driveForward:
                    SM.runOnce(new Runnable() {
                        @Override
                        public void run() {
                            vuforiaVuMarks.deactivateTracking();
                            vuforiaVuMarks.flash(false);
                        }
                    });
                    SM.setNextState(States.squareToWall, HDWaitTypes.EncoderChangeBoth, 230.0);
                    robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                    Log.w("Test", String.valueOf(vuMark));
                    break;
                case squareToWall:
                    SM.setNextState(States.goToColumn, HDWaitTypes.Timer, 1.75);
                    robot.robotDrive.mecanumDrive_Polar(.3, 90, 0, robot.IMU1.getZheading());
                    failsafeTimer.reset();
                    break;
                case goToColumn:
                    SM.setNextState(States.deposit, HDWaitTypes.Timer, 3.5); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                    robot.robotDrive.mecanumDrive_Polar(.25, -90, 0, robot.IMU1.getZheading());
                    break;
                case deposit:
                    SM.setNextState(States.backOut, HDWaitTypes.Timer, 2.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(-.7);
                    robot.robotGlyph.blockKickerOut();
                    break;
                case backOut:
                    SM.setNextState(States.done, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.tankDrive(-.3, -.3);
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
