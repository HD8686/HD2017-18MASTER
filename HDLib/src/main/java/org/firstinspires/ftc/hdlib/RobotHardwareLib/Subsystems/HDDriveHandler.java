package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.General.Alliance;

/**
 * Created by akash on 8/6/2017.
 */

public class HDDriveHandler {

    public enum Side{
        Right,
        Left,
        Back,
    }

    private int gyroRangeMin, gyroRangeMax;

    private double currentError    = 0.0;
    private double previousError   = 0.0;
    private double totalError      = 0.0;
    private double result          = 0.0;

    private double tolerance = 0.0;

    private boolean continuousGyro;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor.RunMode currRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    private Alliance alliance = Alliance.BLUE_ALLIANCE;


    public HDDriveHandler(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight,boolean continuousGyro, int gyroRangeMin, int gyroRangeMax){
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        initMotors();

        this.continuousGyro = continuousGyro;

        this.gyroRangeMax = gyroRangeMax;
        this.gyroRangeMin = gyroRangeMin;

    }

    /**
     * Sets the alliance of the drive train, will reverse all angles if on red alliance, so we only need to program the blue version of a program.
     * @param alliance the alliance that we are on, either RED_ALLIANCE or BLUE_ALLIANCE
     */
    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    private void initMotors(){
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set the run mode of all of the drive train motors
     * @param RunMode Run mode to set them to.
     */
    public void setMode(DcMotor.RunMode RunMode){
        currRunMode = RunMode;
        frontLeft.setMode(RunMode);
        frontRight.setMode(RunMode);
        backLeft.setMode(RunMode);
        backRight.setMode(RunMode);
    }

    /**
     * Reset encoders of drive train motors, then put them back in their original runmodes.
     */
    //Need to transition this to using a library to reset because that motor mode doesn't give power to motors.
    public void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(currRunMode);
    }

    /**
     * Reverse a side of the drive train to make sure both sides are going the same direction.
     * @param reverse The side to reverse
     */
    public void reverseSide(Side reverse){
        if(reverse == Side.Left){
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);
        }else if(reverse == Side.Right){
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
        }else if(reverse == Side.Back){
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    /**
     * Basic tank drive code, power for left side and power for right side.
     * @param leftPower The power to set the left side motors to.
     * @param rightPower The power to set the right side motors to.
     */
    public void tankDrive(double leftPower, double rightPower){
        leftPower = Range.clip(leftPower,-1,1);
        rightPower = Range.clip(rightPower,-1,1);
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    public void motorBreak(){
        tankDrive(0, 0);
    }

    public void resetPIDvalues(){
        currentError    = 0.0;
        previousError   = 0.0;
        totalError      = 0.0;
        tolerance       = 0.0;
        result          = 0.0;
    }

    public double getCurrentPIDResult(){
        return result;
    }

    public double getCurrentError(){ return currentError;}

    public boolean isOnTarget(){
        boolean isOnTarget = false;
        isOnTarget = (Math.abs(currentError) < tolerance);
        return isOnTarget;
    }

    public void gyroTurn(double targetAngle, double p, double i, double d, double ff, double tolerance, double maxSpeed, double minSpeed, double gyroHeading){

        if(maxSpeed < minSpeed){
            throw new IllegalArgumentException("gyroTurn: max speed lower than min speed!");
        }

        this.tolerance = tolerance;

        currentError = targetAngle - gyroHeading;

        if (continuousGyro) {
            double range = gyroRangeMax - gyroRangeMin;
            if (Math.abs(currentError) > (range / 2))  {
                if (currentError > 0)
                    currentError -= range;
                else
                    currentError += range;
            }
        }

        if (i != 0) {
            double estimated_i = (totalError + currentError) * i;
            if (estimated_i < maxSpeed) {
                if (estimated_i > minSpeed) {
                    totalError += currentError;
                } else {
                    totalError = minSpeed/i;
                }
            } else {
                totalError = maxSpeed/ i;
            }
        }

               result = p     * currentError +
                        i  * totalError +
                        d  * (currentError - previousError) +
                        ff     * targetAngle;
        previousError = currentError;

        if((Math.abs(currentError) < tolerance)){
            motorBreak();
        }else {
            tankDrive(result, -result);
        }

    }





}
