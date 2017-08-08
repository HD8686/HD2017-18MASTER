package org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.lang3.math.NumberUtils;
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

    public int getLeftEncoderAverage(){
        return (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2;
    }

    public int getRightEncoderAverage(){
        return (frontRight.getCurrentPosition() + frontRight.getCurrentPosition())/2;
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
     * Set the motors speeds separately, to be used with drive trains that need separate
     * speed values for each motor.
     * @param Speeds Array of motor speeds to be set.
     */
    public void setMotorSpeeds(double[] Speeds){
        frontLeft.setPower(Range.clip(Speeds[0], -1.00000, 1.000000));
        frontRight.setPower(Range.clip(Speeds[1], -1.00000, 1.000000));
        backLeft.setPower(Range.clip(Speeds[2], -1.00000, 1.000000));
        backRight.setPower(Range.clip(Speeds[3], -1.00000, 1.000000));
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


    /**
     * Polar version of Mecanum Drive code meant to be used when programming Autonomous,
     * converted from C++ in FRC's WPILib into Java by us for use in FTC
     * added a magnitudeMax capability so motors never go above 100% power.
     * @param magnitude The amount of power to use on a scale of 0 to 1
     * @param direction The angle to go in, so 0 would be straight, 90 would be right, etc
     * @param rotation Rotation to rotate the robot while going in a direction, on a scale of -1 to 1
     * @param gyroAngle The current yaw angle of the robot
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation, double gyroAngle){
        if(alliance == Alliance.RED_ALLIANCE){
            direction = -direction;
            rotation = -rotation;
        }

        magnitude = magnitude * Math.sqrt(2.0);
        direction = direction - gyroAngle;
        double dirInRad = (direction + 45.0) * Math.PI/180;

        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double Motors[] = new double[4];
        Motors[0] = sinD * magnitude + rotation; //kFrontLeft Motor
        Motors[1] = cosD * magnitude - rotation; //kFrontRight Motor
        Motors[2] = (cosD  * magnitude + rotation); //kRearLeft Motor
        Motors[3] = (sinD * magnitude - rotation); //kRearRight Motor

        double maxMagnitude = Math.abs(NumberUtils.max(Motors));

        if (maxMagnitude > 1.0) {
            for (int i=0; i < Motors.length ; i++) {
                Motors[i] = Motors[i] / maxMagnitude;
            }
        }

        setMotorSpeeds(Motors);
    }

    /**
     * Cartesian Mecanum Drive code meant to be used with Joysticks,
     * converted from C++ in FRC's WPILib into Java by us for use in FTC
     * added a magnitudeMax capability so motors never go above 100% power.
     * @param x X value of joystick to go in
     * @param y Y value of joystick to go in
     * @param rotation The Rotation value, how much it should turn which is always skidsteer
     * @param gyroAngle The current yaw angle of the robot
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {

        double cosA = Math.cos(gyroAngle * (Math.PI / 180.0));
        double sinA = Math.sin(gyroAngle * (Math.PI / 180.0));

        double xIn = x * cosA + y * sinA;
        double yIn = x * sinA - y * cosA;

        double Motors[] = new double[4];
        Motors[0] = xIn + yIn + rotation; //kFrontLeft Motor
        Motors[1] = -xIn + yIn - rotation; //kFrontRight Motor
        Motors[2] = (-xIn + yIn + rotation); //kRearLeft Motor
        Motors[3] = (xIn + yIn - rotation); //kRearRight Motor

        double maxMagnitude = Math.abs(NumberUtils.max(Motors));

        if (maxMagnitude > 1.0) {
            for (int i=0; i < Motors.length ; i++) {
                Motors[i] = Motors[i] / maxMagnitude;
            }
        }

        setMotorSpeeds(Motors);
    }


    /**
     * PID gyro Turn used to turn a tank drive robot to a specefied gyro angle.
     * @param targetAngle Angle to turn to
     * @param p Preportional value for PID controller
     * @param i Integral value for PID controller
     * @param d Derivative value for PID controller
     * @param ff Feed forward value for PID controller
     * @param tolerance Tolerance to end gyro turn
     * @param maxSpeed Maximum Speed that robot should reach
     * @param minSpeed Minimum Speed robot should work (counts negative)
     * @param gyroHeading Current Gyro Z axis heading
     */
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

        if (result > maxSpeed) {
            result = maxSpeed;
        } else if (result < minSpeed) {
            result = minSpeed;
        }

        if((Math.abs(currentError) < tolerance)){
            motorBreak();
        }else {
            tankDrive(result, -result);
        }

    }





}
