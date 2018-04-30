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

public class Red_NonRelic_Multi_Glyph_Jewel implements HDAuto {

    private enum States{
        delay,
        wait,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        hitJewel,
        driveOffStone,
        wait2,
        strafeToCryptobox,
        wait3,
        driveForward,
        openBox,
        lowerBox,
        driveAwayToPush,
        pushInGlyph,
        backAway,
        strafeToGlyphs,
        driveToGlyphs,
        waitToCollect,
        driveToCryptobox,
        strafeToColumn,
        wait4,
        wait5,
        driveForward2,
        openBox2,
        lowerBox2,
        driveAwayToPush2,
        pushInGlyph2,
        backAway2,
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
    double ultrasonicReading;
    public Red_NonRelic_Multi_Glyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

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
                        SM.setNextState(States.driveOffStone, HDWaitTypes.Timer, 0.05);
                    }else{
                        SM.setNextState(States.driveOffStone, HDWaitTypes.Timer, 0.35);
                        robot.robotJewel.hitFront();
                    }
                    break;
                case driveOffStone:
                    SM.setNextState(States.wait2, HDWaitTypes.EncoderChangeBoth, 850.0);
                    if(turnLeft){
                        if(robot.robotDrive.getEncoderAverage() < -50){
                            robot.robotJewel.resetJewel();
                        }
                    }else{
                        robot.robotJewel.resetJewel();
                    }
                    robot.robotDrive.VLF(-0.25, 0.0, 0.01, 2.0, robot.IMU1.getZheading());
                    break;
                case wait2:
                    SM.setNextState(States.strafeToCryptobox, HDWaitTypes.Timer, 1.5);
                    int ultrasonic = robot.leftUS.rawUltrasonic();
                    if(ultrasonic < 200){
                        list.add(((double) robot.leftUS.rawUltrasonic()));
                    }
                    robot.robotDrive.motorBreak();
                    break;
                case strafeToCryptobox:
                    robot.robotGlyph.raiseLiftGate();
                    ultrasonicReading = median(list);
                    if(ultrasonicReading < 20 || ultrasonicReading > 60){
                        ultrasonicReading = 40;
                    }
                    Log.w("ultrasonicReading", String.valueOf(ultrasonicReading));
                    Log.w("enc Calc", String.valueOf(Math.round(0.01963*ultrasonicReading*ultrasonicReading - 24*(ultrasonicReading) + 1412.1094)));
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeIndividualAvg, 0.01963*ultrasonicReading*ultrasonicReading - 24*(ultrasonicReading) + 1412.1094);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeDiag, 0.02956*ultrasonicReading*ultrasonicReading - 24.23*(ultrasonicReading) + 1724.508);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeDiag, 0.01963*ultrasonicReading*ultrasonicReading - 24*(ultrasonicReading) + 1412.1094);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            SM.setNextState(States.wait3, HDWaitTypes.EncoderChangeDiag, -0.01098*ultrasonicReading*ultrasonicReading - 20*(ultrasonicReading) + 935.64);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                    }
                    break;
                case wait3:
                    SM.setNextState(States.driveForward, HDWaitTypes.Timer, .25);
                    robot.robotDrive.motorBreak();
                    break;
                case driveForward:
                    SM.setNextState(States.openBox, HDWaitTypes.EncoderChangeBoth, 75.0);
                    robot.robotGlyph.extendBox();
                    robot.robotDrive.VLF(-.25, 0, 0.01, 2, robot.IMU1.getZheading());
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
                    robot.robotDrive.VLF(.25, 0, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case pushInGlyph:
                    SM.setNextState(States.backAway, HDWaitTypes.EncoderChangeBoth, 275.0);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case backAway:
                    SM.setNextState(States.strafeToGlyphs, HDWaitTypes.EncoderChangeBoth, 150.0);
                    robot.robotDrive.VLF(.25, 0, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case strafeToGlyphs:
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.wait4, HDWaitTypes.EncoderChangeIndividualAvg, 500.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            SM.setNextState(States.wait4, HDWaitTypes.EncoderChangeDiag, 400.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            SM.setNextState(States.wait4, HDWaitTypes.EncoderChangeDiag, 500.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            SM.setNextState(States.wait4, HDWaitTypes.EncoderChangeDiag, 600.0);
                            robot.robotDrive.mecanumDrive_Polar(.25, 90, 0, robot.IMU1.getZheading());
                            break;
                    }
                    break;
                case wait4:
                    SM.setNextState(States.driveToGlyphs, HDWaitTypes.Timer, .25);
                    robot.robotDrive.motorBreak();
                    break;
                case driveToGlyphs:
                    SM.setNextState(States.waitToCollect, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(.50, 0, 0.01, 2, robot.IMU1.getZheading());
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case waitToCollect:
                    SM.setNextState(States.driveToCryptobox, HDWaitTypes.Timer, 1.0);
                    robot.robotDrive.motorBreak();
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case driveToCryptobox:
                    SM.setNextState(States.strafeToColumn, HDWaitTypes.EncoderChangeBoth, 1500.0);
                    robot.robotDrive.VLF(-.50, 0, 0.01, 2, robot.IMU1.getZheading());
                    robot.robotGlyph.setIntakePower(1.0);
                    robot.robotGlyph.startConveyor();
                    break;
                case strafeToColumn:
                    robot.robotGlyph.raiseLiftGate();
                    ultrasonicReading = median(list);
                    /*if(ultrasonicReading < 20 || ultrasonicReading > 60){
                        ultrasonicReading = 40;
                    }*/
                    Log.w("ultrasonicReading", String.valueOf(ultrasonicReading));
                    Log.w("enc Calc", String.valueOf(Math.round(0.01963*ultrasonicReading*ultrasonicReading - 24*(ultrasonicReading) + 1412.1094)));
                    switch (vuMark) {
                        case UNKNOWN:
                            SM.setNextState(States.wait5, HDWaitTypes.EncoderChangeIndividualAvg, 0.01963*ultrasonicReading*ultrasonicReading - 24*(ultrasonicReading) + 1412.1094);
                            robot.robotDrive.mecanumDrive_Polar(.25, -90, 0, robot.IMU1.getZheading());
                            break;
                        case RIGHT:
                            SM.setNextState(States.wait5, HDWaitTypes.EncoderChangeDiag, 0.02956*ultrasonicReading*ultrasonicReading - 24.23*(ultrasonicReading) + 1724.508);
                            robot.robotDrive.mecanumDrive_Polar(.25, -90, 0, robot.IMU1.getZheading());
                            break;
                        case CENTER:
                            SM.setNextState(States.wait5, HDWaitTypes.EncoderChangeDiag, 0.02956*ultrasonicReading*ultrasonicReading - 24.23*(ultrasonicReading) + 1724.508);
                            robot.robotDrive.mecanumDrive_Polar(.25, -90, 0, robot.IMU1.getZheading());
                            break;
                        case LEFT:
                            SM.setNextState(States.wait5, HDWaitTypes.EncoderChangeDiag, -0.01098*ultrasonicReading*ultrasonicReading - 20*(ultrasonicReading) + 935.64);
                            robot.robotDrive.mecanumDrive_Polar(.25, -90, 0, robot.IMU1.getZheading());
                            break;
                    }
                    break;
                case wait5:
                    SM.setNextState(States.driveForward2, HDWaitTypes.Timer, .25);
                    robot.robotDrive.motorBreak();
                    break;
                case driveForward2:
                    SM.setNextState(States.openBox2, HDWaitTypes.EncoderChangeBoth, 75.0);
                    robot.robotGlyph.extendBox();
                    robot.robotDrive.VLF(-.25, 0, 0.01, 2, robot.IMU1.getZheading());
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
                    robot.robotDrive.VLF(.25, 0, 0.01, 2, robot.IMU1.getZheading());
                    break;
                case pushInGlyph2:
                    SM.setNextState(States.backAway2, HDWaitTypes.EncoderChangeBoth, 275.0);
                    robot.robotDrive.tankDrive(-.25, -.25);
                    break;
                case backAway2:
                    SM.setNextState(States.done, HDWaitTypes.EncoderChangeBoth, 150.0);
                    robot.robotDrive.VLF(.25, 0, 0.01, 2, robot.IMU1.getZheading());
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
