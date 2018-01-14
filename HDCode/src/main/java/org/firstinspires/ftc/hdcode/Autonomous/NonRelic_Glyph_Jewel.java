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
import org.firstinspires.ftc.hdlib.Telemetry.HDDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by FIRSTMentor on 9/23/2017.
 */

public class NonRelic_Glyph_Jewel implements HDAuto {

    private HDRobot robot;
    private HDStateMachine SM;
    private HDVuforiaVuMarks vuforiaVuMarks;
    RelicRecoveryVuMark vuMark;
    ElapsedTime failsafeTimer;

    private HDDashboard dashboard;
    private double delay;
    private Alliance alliance;
    private boolean turnLeft;

    private enum States{
        delay,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        turn,
        turnBack,
        driveOffBoard,
        turnAround,
        straightenUp,
        backIntoStone,
        wait1,
        driveForward,
        squareToWall,
        goToColumn,
        deposit,
        backOut,
        turnToPush,
        pushIn,
        driveAway,
        done
    }

    public NonRelic_Glyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

        this.dashboard = dashboard;

        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        vuforiaVuMarks = new HDVuforiaVuMarks(hardwareMap);
        vuforiaVuMarks.activateTracking();

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.setAlliance(alliance);

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
                    SM.setNextState(States.scanVuMark, HDWaitTypes.Timer, delay);
                    break;
                case scanVuMark:
                    SM.setNextState(States.lowerJewelArm, HDWaitTypes.Timer, 6.0);
                    vuMark = vuforiaVuMarks.scan();
                    dashboard.addDiagnosticSpecificTelemetry(1, "Vumark Reading: %s", String.valueOf(vuMark));
                    switch (vuMark) {
                        case UNKNOWN:
                            break;
                        case LEFT:
                            SM.resetValues();
                            SM.setState(States.lowerJewelArm);
                            break;
                        case CENTER:
                            SM.resetValues();
                            SM.setState(States.lowerJewelArm);
                            break;
                        case RIGHT:
                            SM.resetValues();
                            SM.setState(States.lowerJewelArm);
                            break;
                        default:
                            break;
                    }
                    break;
                case lowerJewelArm:
                    vuforiaVuMarks.deactivateTracking();
                    robot.robotGlyph.setLiftPower(0);
                    if(alliance == Alliance.RED_ALLIANCE){
                        robot.robotJewel.lowerLeftServo();
                        if(robot.robotJewel.getLeftColor() != HDJewel.jewelColor.INCONCLUSIVE){
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
                        if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.RED){
                            turnLeft = true;
                            SM.setNextState(States.turn, HDWaitTypes.Timer, 0.05);
                        }else if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.BLUE){
                            turnLeft = false;
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
                    robot.robotJewel.raiseLeftServo();
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveOffBoard:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        SM.setNextState(States.turnAround, HDWaitTypes.EncoderChangeBoth, 1000.0);
                        robot.robotDrive.tankDrive(-.25, -.25);
                    }else {
                        SM.setNextState(States.straightenUp, HDWaitTypes.EncoderChangeBoth, 825.0);
                        robot.robotDrive.tankDrive(.25, .25);
                    }
                    break;
                case turnAround:
                    SM.setNextState(States.backIntoStone, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(180, 0.015, 0.000004, 0.0006, 0.0, 1.0, 0.35, -0.35, robot.IMU1.getZheading());
                    break;
                case straightenUp:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        SM.setNextState(States.backIntoStone, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(180, 0.025, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.backIntoStone, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(0, 0.025, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }
                    break;
                case backIntoStone:
                    SM.setNextState(States.wait1, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.tankDrive(-0.2, -0.2);
                    break;
                case wait1:
                    SM.setNextState(States.driveForward, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.motorBreak();
                    break;
                case driveForward:
                    SM.setNextState(States.squareToWall, HDWaitTypes.EncoderChangeBoth, 240.0);
                    robot.robotDrive.tankDrive(0.25, 0.25);
                    break;
                case squareToWall:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        SM.setNextState(States.goToColumn, HDWaitTypes.Timer, 1.75);
                        robot.robotDrive.mecanumDrive_Polar(.3, -90, 0, -180.0);
                        failsafeTimer.reset();
                    }else{
                        SM.setNextState(States.goToColumn, HDWaitTypes.Timer, 1.75);
                        robot.robotDrive.mecanumDrive_Polar(.3, 90, 0, 0);
                        failsafeTimer.reset();
                    }
                    break;
                case goToColumn:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.15); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, 90, 0, robot.IMU1.getZheading());
                                break;
                            case LEFT:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 3.4); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, 90, 0, robot.IMU1.getZheading());
                                break;
                            case CENTER:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.9); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, 90, 0, robot.IMU1.getZheading());
                                break;
                            case RIGHT:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.15); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, 90, 0, robot.IMU1.getZheading());
                                break;
                            default:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.15); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, 90, 0, robot.IMU1.getZheading());
                                break;
                        }

                    }else{
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.35); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, -90, 0, robot.IMU1.getZheading());
                                break;
                            case LEFT:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.35); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, -90, 0, robot.IMU1.getZheading());
                                break;
                            case CENTER:SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.85); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, -90, 0, robot.IMU1.getZheading());
                                break;
                            case RIGHT:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 3.4); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, -90, 0, robot.IMU1.getZheading());
                                break;
                            default:
                                SM.setNextState(States.deposit, HDWaitTypes.Timer, 2.35); //2.25 for right, 3.5 for left, 2.9 FOR MIDDLE
                                robot.robotDrive.mecanumDrive_Polar(.265, -90, 0, robot.IMU1.getZheading());
                                break;
                        }
                    }
                    break;
                case deposit:
                    SM.setNextState(States.backOut, HDWaitTypes.Timer, 2.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(-.7);
                    robot.robotGlyph.blockKickerOut();
                    break;
                case backOut:
                    SM.setNextState(States.turnToPush, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.tankDrive(-.3, -.3);
                    break;
                case turnToPush:
                    if(alliance == Alliance.RED_ALLIANCE){
                        SM.setNextState(States.pushIn, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(90, 0.015, 0.000004, 0.0006, 0.0, 1.0, 0.35, -0.35, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.pushIn, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(-90, 0.015, 0.000004, 0.0006, 0.0, 1.0, 0.35, -0.35, robot.IMU1.getZheading());
                    }
                    break;
                case pushIn:
                    if(alliance == Alliance.RED_ALLIANCE){
                        SM.setNextState(States.driveAway, HDWaitTypes.Timer, 1.5);
                        robot.robotDrive.mecanumDrive_Polar(.25, 180, 0, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.driveAway, HDWaitTypes.Timer, 1.5);
                        robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                    }
                    break;
                case driveAway:
                    if(alliance == Alliance.RED_ALLIANCE){
                        SM.setNextState(States.done, HDWaitTypes.Timer, 1.25);
                        robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.done, HDWaitTypes.Timer, 1.25);
                        robot.robotDrive.mecanumDrive_Polar(.25, -180, 0, robot.IMU1.getZheading());
                    }
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
