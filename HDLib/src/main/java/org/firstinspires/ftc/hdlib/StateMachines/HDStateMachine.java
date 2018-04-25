package org.firstinspires.ftc.hdlib.StateMachines;



/**
 * Created by Akash on 8/16/2016.
 */


import com.qualcomm.robotcore.hardware.ColorSensor;

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
    private int prevEncoder0 = 0;
    private int prevEncoder1 = 0;
    private int prevEncoder2 = 0;
    private int prevEncoder3 = 0;
    private double changeEncoder0 = 0.0;
    private double changeEncoder1 = 0.0;
    private double changeEncoder2 = 0.0;
    private double changeEncoder3 = 0.0;
    private ColorSensor currColor;
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
    public void setNextState(Object sL, HDWaitTypes typetoWait, Object Argument, Object Argument2, Object Argument3, Object Argument4) {
        if(!waitingActive) {
            currWaitType = typetoWait;
            switch (typetoWait) {
                case Timer:
                    waitingActive = true;
                    timerExpire = HDGeneralLib.getCurrentTimeSeconds() + ((double) Argument);
                    break;
                case ColorToField:
                    waitingActive = true;
                    currColor = ((ColorSensor) Argument);
                    break;
                case ColorToLine:
                    waitingActive = true;
                    currColor = ((ColorSensor) Argument);
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
                    prevEncoder0 = rDrive.getLeftEncoderAverage();
                    prevEncoder1 = rDrive.getRightEncoderAverage();
                    changeEncoder0 = ((double) Argument);
                    break;
                case EncoderChangeRight:
                    waitingActive = true;
                    prevEncoder0 = rDrive.getLeftEncoderAverage();
                    prevEncoder1 = rDrive.getRightEncoderAverage();
                    changeEncoder0 = ((double) Argument);
                    break;
                case EncoderChangeBoth:
                    waitingActive = true;
                    prevEncoder0 = rDrive.getLeftEncoderAverage();
                    prevEncoder1 = rDrive.getRightEncoderAverage();
                    changeEncoder0 = ((double) Argument);
                    break;
                case EncoderChangeIndividual:
                    waitingActive = true;
                    prevEncoder0 = rDrive.frontLeft.getCurrentPosition();
                    prevEncoder1 = rDrive.frontRight.getCurrentPosition();
                    prevEncoder2 = rDrive.backLeft.getCurrentPosition();
                    prevEncoder3 = rDrive.backRight.getCurrentPosition();
                    changeEncoder0 = ((double) Argument);
                    changeEncoder1 = ((double) Argument2);
                    changeEncoder2 = ((double) Argument3);
                    changeEncoder3 = ((double) Argument4);
                    break;
                case EncoderChangeIndividualAvg:
                    waitingActive = true;
                    prevEncoder0 = rDrive.frontLeft.getCurrentPosition();
                    prevEncoder1 = rDrive.frontRight.getCurrentPosition();
                    prevEncoder2 = rDrive.backLeft.getCurrentPosition();
                    prevEncoder3 = rDrive.backRight.getCurrentPosition();
                    changeEncoder0 = ((double) Argument);
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
    public void setNextState(Object sL, HDWaitTypes typetoWait, Object Argument, Object Argument2){
        setNextState(sL, typetoWait, Argument, Argument2, 0, 0);
    }

    /**
     * Use this function to set a condition to wait for, and then switch to the next state which you set
     * @param sL This is the next state you want it to switch to once your end condition has been met
     * @param typetoWait The type of end condition you want, could be Time, Encoder Counts, PID Target(gyroTurn), etc.
     * @param Argument The argument that goes with the end condition you want for example Seconds, Encoder Ticks, Degrees and others.
     */
    public void setNextState(Object sL, HDWaitTypes typetoWait, Object Argument){
        setNextState(sL,typetoWait,Argument, 0, 0, 0);
    }

    /**
     *Use this function to set a condition to wait for, and then switch to the next state which you set
     * @param sL This is the next state you want it to switch to once your end condition has been met
     * @param typetoWait The type of end condition you want, could be Time, Encoder Counts, PID Target(gyroTurn), etc.
     * This function is for the wait types that don't require a argument.
     */
    public void setNextState(Object sL, HDWaitTypes typetoWait){
        setNextState(sL, typetoWait, 0, 0, 0, 0);
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
        prevEncoder0 = 0;
        prevEncoder1 = 0;
        prevEncoder2 = 0;
        prevEncoder3 = 0;
        changeEncoder0 = 0.0;
        changeEncoder1 = 0.0;
        changeEncoder2 = 0.0;
        changeEncoder3 = 0.0;
        currColor = null;
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
                case ColorToLine:
                    if(currColor.blue() > 23 || currColor.red() > 23){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "Color B: %s, Color R: %s", currColor.blue(), currColor.red());
                    }
                    break;
                case ColorToField:
                    if(currColor.blue() < 21 || currColor.red() < 21){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "Color B: %s, Color R: %s", currColor.blue(), currColor.red());
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
                case EncoderChangeIndividual:
                    if(((Math.abs(rDrive.frontLeft.getCurrentPosition() - prevEncoder0)) > changeEncoder0) && ((Math.abs(rDrive.frontRight.getCurrentPosition() - prevEncoder1)) > changeEncoder1) && ((Math.abs(rDrive.backLeft.getCurrentPosition() - prevEncoder2)) > changeEncoder2) && ((Math.abs(rDrive.backRight.getCurrentPosition() - prevEncoder3)) > changeEncoder3)){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"LF Enc. Chg: %s, LB Enc. Chg: %s, RF Enc. Cng: %s, RB Enc. Chg: %s,", (String.valueOf(df.format((Math.abs(rDrive.frontLeft.getCurrentPosition() - prevEncoder0))))), String.valueOf(df.format((Math.abs(rDrive.backLeft.getCurrentPosition() - prevEncoder2)))), String.valueOf(df.format((Math.abs(rDrive.frontRight.getCurrentPosition() - prevEncoder1)))), String.valueOf(df.format((Math.abs(rDrive.backRight.getCurrentPosition() - prevEncoder3)))));
                    }
                    break;
                case EncoderChangeIndividualAvg:
                    if((Math.abs(rDrive.frontLeft.getCurrentPosition() - prevEncoder0) + Math.abs(rDrive.frontRight.getCurrentPosition() - prevEncoder1) + Math.abs(rDrive.backLeft.getCurrentPosition() - prevEncoder2) + Math.abs(rDrive.backRight.getCurrentPosition() - prevEncoder3))/4 > changeEncoder0){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"LF Enc. Chg: %s, LB Enc. Chg: %s, RF Enc. Cng: %s, RB Enc. Chg: %s,", (String.valueOf(df.format((Math.abs(rDrive.frontLeft.getCurrentPosition() - prevEncoder0))))), String.valueOf(df.format((Math.abs(rDrive.backLeft.getCurrentPosition() - prevEncoder2)))), String.valueOf(df.format((Math.abs(rDrive.frontRight.getCurrentPosition() - prevEncoder1)))), String.valueOf(df.format((Math.abs(rDrive.backRight.getCurrentPosition() - prevEncoder3)))));
                    }
                    break;
                case EncoderChangeBoth:
                    if(((Math.abs(rDrive.getLeftEncoderAverage() - prevEncoder0) + Math.abs(rDrive.getRightEncoderAverage() - prevEncoder1))/2) > changeEncoder0){
                        this.resetValues();
                        State = nextState;
                    }else{
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1,"L Enc. Chg: %s, R Enc. Cng: %s", (String.valueOf(df.format(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoder0)))), (String.valueOf(df.format(Math.abs(rDrive.getRightEncoderAverage() - prevEncoder1)))));
                    }
                    break;
                case EncoderChangeLeft:
                    if(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoder0) > changeEncoder0){
                        this.resetValues();
                        State = nextState;
                    }else {
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "L Enc. Chg: %s", String.valueOf(df.format(Math.abs(rDrive.getLeftEncoderAverage() - prevEncoder0))));
                    }
                    break;
                case EncoderChangeRight:
                    if(Math.abs(rDrive.getRightEncoderAverage() - prevEncoder1) > changeEncoder0){
                        this.resetValues();
                        State = nextState;
                    }else {
                        HDDashboard.getInstance().addLibrarySpecificTelemetry(1, "R Enc. Chg: %s", String.valueOf(df.format(Math.abs(rDrive.getRightEncoderAverage() - prevEncoder1))));
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
