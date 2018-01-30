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
 * Created by FIRSTMentor on 1/13/2018.
 */

public class Relic_DoubleGlyph_Jewel implements HDAuto{

    private enum States{
        delay,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        hitJewel,
        turnBack,
        driveOffBoard,
        driveOffBoard2,
        straightenUp,
        readUltrasonic,
        driveToDistance,
        turnToCryptobox,
        driveToCryptobox,
        ejectGlyphs,
        backAway,
        turnToGlyphs,
        driveToGlyphs,
        driveSlowToGlyphs,
        waitToIntake,
        turnBackToCryptobox,
        driveBackToCryptobox,
        ejectGlyphsAgain,
        backAwayAgain,
        turnToGlyphs2,
        driveToGlyphs2,
        driveSlowToGlyphs2,
        waitToIntake2,
        turnBackToCryptobox2,
        driveBackToCryptobox2,
        ejectGlyphsAgain2,
        backAwayAgain2,
        done,
    }

    private HDRobot robot;
    private HDStateMachine SM;
    private HDVuforiaVuMarks vuforiaVuMarks;
    RelicRecoveryVuMark vuMark;
    ElapsedTime failsafeTimer;

    private HDDashboard dashboard;
    private double delay;
    private Alliance alliance;
    private boolean turnLeft;
    private double ultrasonicValue = 0.0;

    public Relic_DoubleGlyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

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
    public void runLoop(double elapsedTime){
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
                            SM.setNextState(States.hitJewel, HDWaitTypes.Timer, 0.05);
                        }else if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.BLUE){
                            turnLeft = false;
                            SM.setNextState(States.hitJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }else{
                        if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.RED){
                            turnLeft = true;
                            SM.setNextState(States.hitJewel, HDWaitTypes.Timer, 0.05);
                        }else if(robot.robotJewel.getLeftColor() == HDJewel.jewelColor.BLUE){
                            turnLeft = false;
                            SM.setNextState(States.hitJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }
                    break;
                case hitJewel:
                    if(turnLeft){
                        SM.setNextState(States.turnBack, HDWaitTypes.Timer, 0.15);
                     }else{
                        SM.setNextState(States.turnBack, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(12, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }
                    break;
                case turnBack:
                    SM.setNextState(States.driveOffBoard, HDWaitTypes.driveHandlerTarget);
                    if(!turnLeft)
                        robot.robotJewel.raiseLeftServo();
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveOffBoard:
                    SM.setNextState(States.driveOffBoard2, HDWaitTypes.EncoderChangeBoth, 200.0);
                    if(alliance == Alliance.RED_ALLIANCE)
                        robot.robotDrive.tankDrive(-.25, -.25);
                    else
                        robot.robotDrive.tankDrive(.25, .25);
                    break;
                case driveOffBoard2:
                    SM.setNextState(States.straightenUp, HDWaitTypes.EncoderChangeBoth, 700.0);
                    robot.robotJewel.raiseLeftServo();
                    if(alliance == Alliance.RED_ALLIANCE)
                        robot.robotDrive.tankDrive(-.25, -.25);
                    else
                        robot.robotDrive.tankDrive(.25, .25);
                    break;
                case straightenUp:
                    SM.setNextState(States.readUltrasonic, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case readUltrasonic:
                    SM.setNextState(States.driveToDistance, HDWaitTypes.Timer, 0.25);
                    if(alliance == Alliance.RED_ALLIANCE)
                        ultrasonicValue = robot.frontUS.getDistanceCM();
                    else
                        ultrasonicValue = robot.backUS.getDistanceCM();
                    Log.w("Test", String.valueOf(ultrasonicValue));
                    break;
                case driveToDistance:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0)+0.15);
                                robot.robotDrive.tankDrive(-.25, -.25);
                                break;
                            case LEFT:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0));
                                robot.robotDrive.tankDrive(-.25, -.25);
                                break;
                            case CENTER: //160
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0));
                                robot.robotDrive.tankDrive(-.25, -.25);
                                break;
                            case RIGHT: //140
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-1.0/38.0)*(ultrasonicValue)+(70.0/19.0)+0.15);
                                robot.robotDrive.tankDrive(-.25, -.25);
                                break;
                            default:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0)+0.15);
                                robot.robotDrive.tankDrive(-.25, -.25);
                                break;
                        }
                    }else{
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0)+0.15);
                                robot.robotDrive.tankDrive(0.25, 0.25);
                                break;
                            case LEFT:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-1.0/38.0)*(ultrasonicValue)+(70.0/19.0)+0.15);
                                robot.robotDrive.tankDrive(0.25, 0.25);
                                break;
                            case CENTER:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0));
                                robot.robotDrive.tankDrive(0.25, 0.25);
                                break;
                            case RIGHT:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0));
                                robot.robotDrive.tankDrive(0.25, 0.25);
                                break;
                            default:
                                SM.setNextState(States.turnToCryptobox, HDWaitTypes.Timer, (-3.0/116.0)*(ultrasonicValue)+(120.0/29.0)+0.15);
                                robot.robotDrive.tankDrive(0.25, 0.25);
                                break;
                        }
                    }
                    break;
                case turnToCryptobox:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case LEFT:
                                SM.setNextState(States.ejectGlyphs, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-90, 0.0095, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case CENTER: //160
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-52, 0.0085, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case RIGHT: //140
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            default:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                        }
                    }else{
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case LEFT:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case CENTER:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(52, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case RIGHT:
                                SM.setNextState(States.ejectGlyphs, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(90, 0.0095, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            default:
                                SM.setNextState(States.driveToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(50, 0.01, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                        }
                    }
                    break;
                case driveToCryptobox:
                    SM.setNextState(States.ejectGlyphs, HDWaitTypes.Timer, 0.65);
                    robot.robotDrive.tankDrive(.25, .25);
                    break;
                case ejectGlyphs:
                    SM.setNextState(States.backAway, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(-.7);
                    robot.robotGlyph.blockKickerOut();
                    break;
                case backAway:
                    SM.setNextState(States.turnToGlyphs, HDWaitTypes.Timer, .5);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case turnToGlyphs:
                    if(alliance == Alliance.RED_ALLIANCE) {
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.driveToGlyphs, HDWaitTypes.driveHandlerTarget);
                            robot.robotDrive.gyroTurn(80, 0.0065, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                            robot.robotGlyph.blockKickerIn();
                            break;
                        case LEFT:

                            break;
                        case CENTER:
                            SM.setNextState(States.driveToGlyphs, HDWaitTypes.driveHandlerTarget);
                            robot.robotDrive.gyroTurn(80, 0.0065, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                            robot.robotGlyph.blockKickerIn();
                            break;
                        case RIGHT:

                             break;
                        default:
                            SM.setNextState(States.driveToGlyphs, HDWaitTypes.driveHandlerTarget);
                            robot.robotDrive.gyroTurn(80, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                            robot.robotGlyph.blockKickerIn();
                            break;
                    }
                }else{
                    switch (vuMark) {
                        case UNKNOWN:

                            break;
                        case LEFT:

                            break;
                        case CENTER:

                            break;
                        case RIGHT:

                            break;
                        default:

                            break;
                    }
                }
                    break;
                case driveToGlyphs:
                    SM.setNextState(States.driveSlowToGlyphs, HDWaitTypes.Timer, 1.15);
                    robot.robotDrive.tankDrive(.5, .5);
                    robot.robotGlyph.setIntakePower(.7);
                    break;
                case driveSlowToGlyphs:
                    SM.setNextState(States.waitToIntake, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.tankDrive(.25, .25);
                    break;
                case waitToIntake:
                    SM.setNextState(States.turnBackToCryptobox, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    break;
                case turnBackToCryptobox:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.driveBackToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-87, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());

                                break;
                            case LEFT:

                                break;
                            case CENTER:
                                SM.setNextState(States.driveBackToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-86, 0.0065, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());

                                break;
                            case RIGHT:

                                break;
                            default:
                                SM.setNextState(States.driveBackToCryptobox, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-87, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());

                                break;
                        }
                    }else{
                        switch (vuMark) {
                            case UNKNOWN:

                                break;
                            case LEFT:

                                break;
                            case CENTER:

                                break;
                            case RIGHT:

                                break;
                            default:

                                break;
                        }
                    }
                        break;
                case driveBackToCryptobox:
                    SM.setNextState(States.ejectGlyphsAgain, HDWaitTypes.Timer, 1.55);
                    robot.robotDrive.tankDrive(.5, .5);
                    break;
                case ejectGlyphsAgain:
                    SM.setNextState(States.backAwayAgain, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(-.7);
                    robot.robotGlyph.blockKickerOut();
                    break;
                case backAwayAgain:
                    SM.setNextState(States.turnToGlyphs2, HDWaitTypes.Timer, 0.35);
                    robot.robotDrive.tankDrive(-.5, -.5);
                    break;
                case turnToGlyphs2:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.driveToGlyphs2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(70, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                robot.robotGlyph.blockKickerIn();
                                break;
                            case LEFT:

                                break;
                            case CENTER:
                                SM.setNextState(States.driveToGlyphs2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(70, 0.0065, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                robot.robotGlyph.blockKickerIn();
                                break;
                            case RIGHT:

                                break;
                            default:
                                SM.setNextState(States.driveToGlyphs2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(70, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                robot.robotGlyph.blockKickerIn();
                                break;
                        }
                    }else{
                        switch (vuMark) {
                            case UNKNOWN:

                                break;
                            case LEFT:

                                break;
                            case CENTER:

                                break;
                            case RIGHT:

                                break;
                            default:

                                break;
                        }
                    }
                    break;
                case driveToGlyphs2:
                    SM.setNextState(States.driveSlowToGlyphs2, HDWaitTypes.Timer, 1.25);
                    robot.robotDrive.tankDrive(.5, .5);
                    robot.robotGlyph.setIntakePower(.7);
                    break;
                case driveSlowToGlyphs2:
                    SM.setNextState(States.waitToIntake2, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.tankDrive(.25, .25);
                    break;
                case waitToIntake2:
                    SM.setNextState(States.turnBackToCryptobox2, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    break;
                case turnBackToCryptobox2:
                    if(alliance == Alliance.RED_ALLIANCE) {
                        switch (vuMark) {
                            case UNKNOWN:
                                SM.setNextState(States.driveBackToCryptobox2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-80, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case LEFT:

                                break;
                            case CENTER:
                                SM.setNextState(States.driveBackToCryptobox2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-80, 0.0065, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                            case RIGHT:

                                break;
                            default:
                                SM.setNextState(States.driveBackToCryptobox2, HDWaitTypes.driveHandlerTarget);
                                robot.robotDrive.gyroTurn(-80, 0.008, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                                break;
                        }
                    }else{
                        switch (vuMark) {
                            case UNKNOWN:

                                break;
                            case LEFT:

                                break;
                            case CENTER:

                                break;
                            case RIGHT:

                                break;
                            default:

                                break;
                        }
                    }
                    break;
                case driveBackToCryptobox2:
                    SM.setNextState(States.ejectGlyphsAgain2, HDWaitTypes.Timer, 1.6);
                    robot.robotDrive.tankDrive(.5, .5);
                    break;
                case ejectGlyphsAgain2:
                    SM.setNextState(States.backAwayAgain2, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(-.7);
                    robot.robotGlyph.blockKickerOut();
                    break;
                case backAwayAgain2:
                    SM.setNextState(States.done, HDWaitTypes.Timer, 0.5);
                    robot.robotDrive.tankDrive(-.5, -.5);
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
