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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by baseball0206 on 4/24/2018.
 */

public class Red_NonRelic_DoubleGlyph_Jewel implements HDAuto{

    private enum States{
        delay,
        wait,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        hitJewel,
        strafeOffStone,
        wait2,
        turnToGlyphs,
        readUltrasonic,
        driveToGlyphs,
        adjustGlyphs1,
        driveBack,
        turnToCryptobox,
        readUltrasonic2,
        driveToCryptobox,
        readUltrasonic3,
        strafeToColumn,
        wait3,
        driveForward,
        openBox,
        lowerBox,
        driveAwayToPush,
        pushInGlyph,
        backAway,
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
    private Red_NonRelic_DoubleGlyph_Jewel.States nextState;

    List<Double> list = new ArrayList<>();
    public Red_NonRelic_DoubleGlyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

        this.dashboard = dashboard;

        robot = new HDRobot(hardwareMap);
        timer = new ElapsedTime();
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
            switch (states) {
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
                        SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 0.35);
                        robot.robotJewel.hitBack();
                    }else{
                        SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 0.35);
                        robot.robotJewel.hitFront();
                    }
                    break;
                case strafeOffStone:
                    SM.setNextState(States.wait2, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.mecanumDrive_Polar(0.5,90, 0, robot.IMU1.getZheading());
                    break;
                case wait2:
                    SM.setNextState(States.turnToGlyphs, HDWaitTypes.Timer, .25);
                    robot.robotDrive.motorBreak();
                    break;
                case turnToGlyphs:
                    SM.setNextState(States.readUltrasonic, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(45, 0.0085, 0.000004, 0.0006, 0.00, 2.0, 1.0, -1.0, robot.IMU1.getZheading());;
                    timer.reset();
                    break;
                case readUltrasonic:
                    SM.setNextState(States.driveToGlyphs, HDWaitTypes.Timer, 0.5);
                    if (timer.milliseconds() > 90 ) {
                        double reading = robot.frontUS.getDistance(DistanceUnit.CM);

                        if (reading < 150) {
                            list.add(reading);
                        } else {

                        }

                        average = median(list);

                        timer.reset();
                    }
                    robot.robotDrive.motorBreak();
                    break;
                case driveToGlyphs:
                    SM.setNextState(States.adjustGlyphs1,HDWaitTypes.EncoderChangeBoth, (average<150 && average>20)?(average*ENCODERS_PER_CM):61*ENCODERS_PER_CM);
                    robot.robotDrive.VLF(0.5, 45.0, .01, 2, robot.IMU1.getZheading());
                    robot.robotGlyph.openBox();
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case adjustGlyphs1:
                    SM.setNextState(States.driveBack, HDWaitTypes.Timer, .25);
                    robot.robotGlyph.setIntakePower(-1.0);
                    robot.robotGlyph.backwardsConveyor();
                    robot.robotDrive.motorBreak();
                    break;
                case driveBack:
                    SM.setNextState(States.turnToCryptobox, HDWaitTypes.EncoderChangeBoth, (average<150 && average>20)?(average*ENCODERS_PER_CM):61*ENCODERS_PER_CM);
                    robot.robotDrive.VLF(-0.5, 45.0, .01, 2, robot.IMU1.getZheading());
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case turnToCryptobox:
                    if(robot.robotDrive.isOnTarget()){
                        waitBeforeNextState(0.3, States.driveForward);
                    }
                    robot.robotGlyph.setIntakePower(0.0);
                    robot.robotGlyph.stopConveyor();
                    robot.robotDrive.gyroTurn(0, 0.0085, 0.000004, 0.0006, 0.00, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    timer.reset();
                    list.clear();
                    break;
                case readUltrasonic2:
                    SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 0.5);
                    if (timer.milliseconds() > 90 ) {
                        double reading = robot.backUS.getDistance(DistanceUnit.CM);

                        if (reading < 200) {
                            list.add(reading);
                        } else {

                        }

                        average = median(list);

                        timer.reset();
                    }
                    robot.robotDrive.motorBreak();
                    break;
                case driveToCryptobox:
                    SM.setNextState(States.turnToCryptobox, HDWaitTypes.EncoderChangeBoth, (average<200 && average>20)?((average-25)*ENCODERS_PER_CM):92*ENCODERS_PER_CM);
                    robot.robotDrive.VLF(-0.5, 45.0, .01, 2, robot.IMU1.getZheading());
                    list.clear();
                    timer.reset();
                    break;
                case readUltrasonic3:
                    SM.setNextState(States.strafeToColumn, HDWaitTypes.Timer, 0.5);
                    if (timer.milliseconds() > 90 ) {
                        double reading = robot.leftUS.getDistance(DistanceUnit.CM);

                        if (reading < 150) {
                            list.add(reading);
                        } else {

                        }

                        average = median(list);

                        timer.reset();
                    }
                    robot.robotDrive.motorBreak();
                    break;
                case strafeToColumn:
                    robot.robotGlyph.gripBox();
                    robot.robotDrive.mecanumDrive_Polar(.5, -90, 0.0, robot.IMU1.getZheading());
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeIndividual, (average<200 && average>20)?((average-92)*ENCODERS_PER_CM):31*ENCODERS_PER_CM);
                            break;
                        case LEFT:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeIndividual, (average<200 && average>20)?((average-107)*ENCODERS_PER_CM):16*ENCODERS_PER_CM);
                            break;
                        case CENTER:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeIndividual, (average<200 && average>20)?((average-92)*ENCODERS_PER_CM):31*ENCODERS_PER_CM);
                            break;
                        case RIGHT:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeIndividual, (average<200 && average>20)?((average-77)*ENCODERS_PER_CM):61*ENCODERS_PER_CM);
                            break;
                    }
                    break;
                case wait3:
                    SM.setNextState(States.driveForward, HDWaitTypes.Timer, .5);
                    robot.robotGlyph.raiseLiftGate();
                    robot.robotDrive.motorBreak();
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
