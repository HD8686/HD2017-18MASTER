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

import java.util.ArrayList;
import java.util.List;

/**
 * Created by FIRSTMentor on 1/13/2018.
 */

public class Red_Relic_Multi_Glyph_Jewel implements HDAuto {

    private enum States{
        delay,
        wait,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        hitJewel,
        driveToCryptobox,
        turnToCryptobox,
        driveForward,
        openBox,
        lowerBox,
        driveAwayToPush,
        pushInGlyph,
        backAway,
        driveToGlyphs,
        waitToCollect,
        strafeToColumn,
        driveToCryptobox2,
        driveForward2,
        openBox2,
        lowerBox2,
        driveAwayToPush2,
        pushInGlyph2,
        backAway2,
        driveToGlyphs2,
        waitToCollect2,
        strafeToColumn2,
        driveToCryptobox3,
        driveForward3,
        openBox3,
        lowerBox3,
        driveAwayToPush3,
        pushInGlyph3,
        backAway3,
        done,
    }

    private HDRobot robot;
    private HDStateMachine SM;
    private HDVuforiaVuMarks vuforiaVuMarks;
    private RelicRecoveryVuMark vuMark;

    private HDDashboard dashboard;
    private double delay, waitTime, average;
    private boolean turnLeft;
    private Alliance alliance;
    private ElapsedTime timer;
    private final double ENCODERS_PER_CM = 45.115;
    private States nextState;
    List<Double> list = new ArrayList<>();
    public Red_Relic_Multi_Glyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

        this.dashboard = dashboard;

        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        vuforiaVuMarks = new HDVuforiaVuMarks(hardwareMap);
        vuforiaVuMarks.activateTracking();

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.setAlliance(alliance);

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
    public void initializeLoop() {
        dashboard.addDiagnosticSpecificTelemetry(1, "Vumark Reading: %s", String.valueOf(vuforiaVuMarks.scan()));
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
                case wait:
                    SM.setNextState(nextState, HDWaitTypes.Timer, waitTime);
                    robot.robotDrive.resetEncoders();
                    break;
                case scanVuMark:
                    SM.setNextState(States.lowerJewelArm, HDWaitTypes.Timer, 6.0);
                    robot.robotGlyph.gripBox();
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
                    robot.robotJewel.lowerArm();
                    if(robot.robotJewel.getLeftColor() != HDJewel.jewelColor.INCONCLUSIVE){
                        SM.setNextState(States.readJewel, HDWaitTypes.Timer, 0.05);
                    }
                    break;
                case readJewel:
                    Log.w("Jewel Color", String.valueOf(robot.robotJewel.getLeftColor()));
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
                    Log.w("turnLeft", String.valueOf(turnLeft));
                    if(turnLeft){
                        SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 0.05);
                    }else{
                        SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 0.35);
                        robot.robotJewel.hitFront();
                    }
                    break;
                case driveToCryptobox:
                    if(turnLeft){
                        if(robot.robotDrive.getEncoderAverage() < -50){
                            robot.robotJewel.resetJewel();
                        }
                    }else{
                        robot.robotJewel.resetJewel();
                    }
                    int targetEncoder, error;
                    double percentCompleted;
                    switch (vuMark) {
                        case UNKNOWN:
                            targetEncoder = -1495;
                            error = targetEncoder - robot.robotDrive.getEncoderAverage();
                            percentCompleted = Math.abs(((double) error)/((double) Math.abs(targetEncoder)));
                            if(percentCompleted > .75){percentCompleted = 0.65;}
                            robot.robotDrive.VLF(-((percentCompleted > 0.1) ? percentCompleted : 0.1), 0, 0.01, 2,robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            targetEncoder = -1840;
                            error = targetEncoder - robot.robotDrive.getEncoderAverage();
                            percentCompleted = Math.abs(((double) error)/((double) Math.abs(targetEncoder)));
                            if(percentCompleted > .75){percentCompleted = 0.65;}
                            robot.robotDrive.VLF(-((percentCompleted > 0.1) ? percentCompleted : 0.1), 0, 0.01, 2,robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            targetEncoder = -1500;
                            error = targetEncoder - robot.robotDrive.getEncoderAverage();
                            percentCompleted = Math.abs(((double) error)/((double) Math.abs(targetEncoder)));
                            if(percentCompleted > .75){percentCompleted = 0.65;}
                            robot.robotDrive.VLF(-((percentCompleted > 0.1) ? percentCompleted : 0.1), 0, 0.01, 2,robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            targetEncoder = -1115;
                            error = targetEncoder - robot.robotDrive.getEncoderAverage();
                            percentCompleted = Math.abs(((double) error)/((double) Math.abs(targetEncoder)));
                            if(percentCompleted > .75){percentCompleted = 0.65;}
                            robot.robotDrive.VLF(-((percentCompleted > 0.1) ? percentCompleted : 0.1), 0, 0.01, 2,robot.IMU1.getZheading());
                            break;
                        default:
                            targetEncoder = -1500;
                            error = targetEncoder - robot.robotDrive.getEncoderAverage();
                            percentCompleted = Math.abs(((double) error)/((double) Math.abs(targetEncoder)));
                            if(percentCompleted > .75){percentCompleted = 0.65;}
                            robot.robotDrive.VLF(-((percentCompleted > 0.1) ? percentCompleted : 0.1), 0, 0.01, 2,robot.IMU1.getZheading());
                            break;
                    }
                    Log.w("error", String.valueOf(error));
                    if(error > -10){
                        robot.robotDrive.motorBreak();
                        waitBeforeNextState(0.3, States.turnToCryptobox);
                    }
                    break;
                case turnToCryptobox:
                    if(robot.robotDrive.isOnTarget()){
                        waitBeforeNextState(0.3, States.driveForward);
                    }
                    robot.robotGlyph.raiseLiftGate();
                    robot.robotDrive.gyroTurn(90, 0.0085, 0.000004, 0.0006, 0.00, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveForward:
                    SM.setNextState(States.openBox, HDWaitTypes.EncoderChangeBoth, 150.0);
                    robot.robotGlyph.extendBox();
                    robot.robotDrive.VLF(-.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case openBox:
                    SM.setNextState(States.lowerBox, HDWaitTypes.Timer, 1.5);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.openBox();
                    break;
                case lowerBox:
                    SM.setNextState(States.driveAwayToPush, HDWaitTypes.Timer, .25);
                    robot.robotGlyph.stowBox();
                    break;
                case driveAwayToPush:
                    SM.setNextState(States.pushInGlyph, HDWaitTypes.EncoderChangeBoth, 100.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case pushInGlyph:
                    SM.setNextState(States.backAway, HDWaitTypes.EncoderChangeBoth, 215.0);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case backAway:
                    SM.setNextState(States.driveToGlyphs, HDWaitTypes.EncoderChangeBoth, 250.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case driveToGlyphs:
                    SM.setNextState(States.waitToCollect, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(.50, 90, 0.01, 2, robot.IMU1.getZheading());
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case waitToCollect:
                    SM.setNextState(States.strafeToColumn, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case strafeToColumn:
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.driveToCryptobox2, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 180, 0, robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            SM.setNextState(States.driveToCryptobox2, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            SM.setNextState(States.driveToCryptobox2, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 180, 0, robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            SM.setNextState(States.driveToCryptobox2, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 180, 0, robot.IMU1.getZheading());
                            break;
                    }
                    break;
                case driveToCryptobox2:
                    SM.setNextState(States.driveForward2, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(-.50, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case driveForward2:
                    SM.setNextState(States.openBox2, HDWaitTypes.EncoderChangeBoth, 150.0);
                    robot.robotGlyph.extendBox();
                    robot.robotDrive.VLF(-.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case openBox2:
                    SM.setNextState(States.lowerBox2, HDWaitTypes.Timer, 1.5);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.openBox();
                    break;
                case lowerBox2:
                    SM.setNextState(States.driveAwayToPush2, HDWaitTypes.Timer, .25);
                    robot.robotGlyph.stowBox();
                    break;
                case driveAwayToPush2:
                    SM.setNextState(States.pushInGlyph2, HDWaitTypes.EncoderChangeBoth, 100.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case pushInGlyph2:
                    SM.setNextState(States.backAway2, HDWaitTypes.EncoderChangeBoth, 215.0);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case backAway2:
                    SM.setNextState(States.driveToGlyphs2, HDWaitTypes.EncoderChangeBoth, 250.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case driveToGlyphs2:
                    SM.setNextState(States.waitToCollect2, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(.50, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case waitToCollect2:
                    SM.setNextState(States.strafeToColumn2, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case strafeToColumn2:
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.driveToCryptobox3, HDWaitTypes.EncoderChangeIndividualAvg, 540.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            SM.setNextState(States.driveToCryptobox3, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            SM.setNextState(States.driveToCryptobox3, HDWaitTypes.EncoderChangeIndividualAvg, 540.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 0, 0, robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            SM.setNextState(States.driveToCryptobox3, HDWaitTypes.EncoderChangeIndividualAvg, 225.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 180, 0, robot.IMU1.getZheading());
                            break;
                    }
                    break;
                case driveToCryptobox3:
                    SM.setNextState(States.driveForward3, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(-.50, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case driveForward3:
                    SM.setNextState(States.openBox3, HDWaitTypes.EncoderChangeBoth, 150.0);
                    robot.robotGlyph.extendBox();
                    robot.robotDrive.VLF(-.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case openBox3:
                    SM.setNextState(States.lowerBox3, HDWaitTypes.Timer, 1.5);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.openBox();
                    break;
                case lowerBox3:
                    SM.setNextState(States.driveAwayToPush3, HDWaitTypes.Timer, .25);
                    robot.robotGlyph.stowBox();
                    break;
                case driveAwayToPush3:
                    SM.setNextState(States.pushInGlyph3, HDWaitTypes.EncoderChangeBoth, 100.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case pushInGlyph3:
                    SM.setNextState(States.backAway3, HDWaitTypes.EncoderChangeBoth, 215.0);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case backAway3:
                    SM.setNextState(States.done, HDWaitTypes.EncoderChangeBoth, 250.0);
                    robot.robotDrive.VLF(.25, 90, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case done:
                    robot.robotGlyph.setIntakePower(0.0);
                    robot.robotGlyph.lowerGlyphGate();
                    robot.robotDrive.motorBreak();
                    break;
            }
        }
    }

    public void waitBeforeNextState(double waitTime, States nextState){
        this.nextState = nextState;
        this.waitTime = waitTime;
        SM.resetValues();
        SM.setState(States.wait);
    }

    private boolean ready(){
        boolean ready = true;
        if(!robot.IMU1.isCalibrated()){
            ready = false;
        }
        return ready;
    }
    public double median(List<Double> a) {
        int middle = a.size() / 2;

        if (a.size() % 2 == 1) {
            return a.get(middle);
        } else {
            return (a.get(middle - 1) + a.get(middle)) / 2.0;
        }
    }
}
