package org.firstinspires.ftc.hdlib.StateMachines;



/**
 * Created by Akash on 8/16/2016.
 */


import org.firstinspires.ftc.hdlib.General.HDGeneralLib;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.hdlib.Sensors.HDMROpticalDistance;
import org.firstinspires.ftc.hdlib.Sensors.HDMRRange;
import org.firstinspires.ftc.hdlib.Telemetry.HDDashboard;

import java.text.DecimalFormat;

/**
 * This is our Height Differential state machine library.
 * It controls the state of Autonomous and general robot checks like gyro(either NavX or Modern Robotics) calibration which is why we use it in Teleop and Autonomous.
 * The purpose of this library is to make it very easy to define end conditions, and as they are developed they are easy to reuse.
 */
public class HDStateMachine {

    private Object State;
    private Object nextState;
    private HDMROpticalDistance currODS;
    private HDMRRange currRange;
    private boolean waitingActive = false;
    private double timerExpire = 0.0;
    private double targetRange = 0.0;
    private boolean hasRun = false;
    private HDDriveHandler rDrive;
    private HDWaitTypes currWaitType = HDWaitTypes.Nothing;
    private DecimalFormat df;
    private int prevEncoderLeft = 0;
    private int prevEncoderRight = 0;
    private double changeEncoder = 0.0;
    private Runnable reset;

    /**
     * This is the definition for the State Machine Class
     * @param robotD The Robot Drive handler for the robot, which controls the drive base.
     */
    public HDStateMachine(HDDriveHandler robotD){
        this.rDrive = robotD;
        df = new DecimalFormat("#.##");
    }

    /**
     * Use this function to set the current state of the state machine
     * @param sL This is the state that you want to set the state machine to
     */
    public void setState(Object sL){
        State = sL;
        resetValues();
    }

    /**
     * Use this function to set a condition to wait for, and then switch to the next state which you set
     * @param sL This is the next state you want it to switch to once your end condition has been met
     * @param typetoWait The type of end condition you want, could be Time, Encoder Counts, PID Target(gyroTurn), etc.
     * @param Argument The argument that goes with the end condition you want for example Seconds, Encoder Ticks, Degrees and others.
     */
    public void setNextState(Object sL, HDWaitTypes typetoWait, Object Argument, Object Argument2){
        if(!waitingActive) {
            currWaitType = typetoWait;
            switch (typetoWait) {
                case Timer:
                    waitingActive = true;
                    timerExpire = HDGeneralLib.getCurrentTimeSeconds() + ((double) Argument);
                    break;
                case ODStoLine:
                    waitingActive = true;
                    currODS = ((HDMROpticalDistance) Argument);
                    break;
                case ODStoField:
                    waitingActive = true;
                    currODS = ((HDMROpticalDistance) Argument);
                    break;
                case Range:
                    waitingActive = true;
                    currRange = ((HDMRRange) Argument);
                    targetRange = ((double) Argument2);
                    break;
                case EncoderChangeLeft:
                    waitingActive = true;
                    prevEncoderLeft = rDrive.getLeftEncoderAverage();
                    prevEncoderRight = rDrive.getRightEncoderAverage();
                    changeEncoder = ((double) Argument);
                    break;
                case EncoderChangeRight:
                    waitingActive = true;
                    prevEncoderLeft = rDrive.getLeftEncoderAverage();
                    prevEncoderRight = rDrive.getRightEncoderAverage();
                    changeEncoder = ((double) Argument);
                    break;
                case EncoderChangeBoth:
                    waitingActive = true;
                    prevEncoderLeft = rDrive.getLeftEncoderAverage();
                    prevEncoderRight = rDrive.getRightEncoderAverage();
                    changeEncoder = ((double) Argument);
                    break;
                case driveHandlerTarget:
                    waitingActive = true;
                    break;
                case Nothing:
                    break;
            }
            nextState = sL;
        }
    }

    /**
     * Use this function to set a condition to wait for, and then switch to the next state which you set
     * @param sL This is the next state you want it to switch to once your end condition has been met
     * @param typetoWait The type of end condition you want, could be Time, Encoder Counts, PID Target(gyroTurn), etc.
     * @param Argument The argument that goes with the end condition you want for example Seconds, Encoder Ticks, Degrees and others.
     */
    public void setNextState(Object sL, HDWaitTypes typetoWait, Object Argument){
        setNextState(sL,typetoWait,Argument, 0);
    }

    /**
     *Use this function to set a condition to wait for, and then switch to the next state which you set
     * @param sL This is the next state you want it to switch to once your end condition has been met
     * @param typetoWait The type of end condition you want, could be Time, Encoder Counts, PID Target(gyroTurn), etc.
     * This function is for the wait types that don't require a argument.
     */
    public void setNextState(Object sL, HDWaitTypes typetoWait){
        setNextState(sL, typetoWait, 0);
    }


    public void resetValues()
    {
        waitingActive = false;
        timerExpire = 0.0;
        currODS = null;
        currRange = null;
        currWaitType = HDWaitTypes.Nothing;
        hasRun = false;
        targetRange = 0.0;
        prevEncoderLeft = 0;
        prevEncoderRight = 0;
        changeEncoder = 0.0;
        this.reset.run();
    }

    public void setResetCode(Runnable code){
        this.reset = code;
    }

    public void runOnce(Runnable code){
        if(!hasRun){
            code.run();
            hasRun = true;
        }
    }

    public Object getState(){
        if(this.waitingActive){
            switch(this.currWaitType){
                case Timer:
                    if(this.timerExpire <= HDGeneralLib.getCurrentTimeSeconds()){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"Delay Left: " + (String.valueOf(df.format(this.timerExpire - HDGeneralLib.getCurrentTimeSeconds()))));
                        }
                    break;
                case ODStoLine:
                    if(currODS.getRawLightDetected() > .3){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"ODS Value: " + (String.valueOf(df.format(currODS.getRawLightDetected()))));
                    }
                    break;
                case ODStoField:
                    if(currODS.getRawLightDetected() < .3){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"ODS Value: " + (String.valueOf(df.format(currODS.getRawLightDetected()))));
                    }
                    break;
                case Range:
                    if(currRange.getUSValue() == targetRange){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"Range_Button_Pusher Value: " + (String.valueOf(currRange.getUSValue())));
                    }
                    break;
                case EncoderChangeBoth:
                    if(((Math.abs(rDrive.getLeftEncoderAverage() - prevEncoderLeft) + Math.abs(rDrive.getRightEncoderAverage() - prevEncoderRight))/2) > changeEncoder){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"L Enc. Chg: %s, R Enc. Cng: %s", (String.valueOf(df.format(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoderLeft)))), (String.valueOf(df.format(Math.abs(rDrive.getRightEncoderAverage() - prevEncoderRight)))));
                    }
                    break;
                case EncoderChangeLeft:
                    if(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoderLeft) > changeEncoder){
                        this.resetValues();
                        State = nextState;
                    }else {
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "L Enc. Chg: %s", String.valueOf(df.format(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoderLeft))));
                    }
                    break;
                case EncoderChangeRight:
                    if(Math.abs(rDrive.getRightEncoderAverage() - prevEncoderRight) > changeEncoder){
                        this.resetValues();
                        State = nextState;
                    }else {
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "R Enc. Chg: %s", String.valueOf(df.format(Math.abs(rDrive.getRightEncoderAverage() - prevEncoderRight))));
                    }
                    break;
                case driveHandlerTarget:
                    if(rDrive.isOnTarget()){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "Current DH Error: %s", String.valueOf(df.format(rDrive.getCurrentError())));
                    }
                    break;
                case Nothing:
                    this.resetValues();
                    State = nextState;
                    break;
            }

        }
        HDDashboard.getInstance().addLibrarySpecificTelemetry(0,"Current State Running: " + State.toString());
        return State;
    }
}
