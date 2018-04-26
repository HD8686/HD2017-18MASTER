package org.firstinspires.ftc.hdcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.General.TeleopEnum;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class HDTeleop extends HDOpMode implements HDGamepad.HDButtonMonitor{

    private HDGamepad driverGamepad;
    private HDGamepad servoBoyGamepad;
    private HDRobot robot;
    private double lastSpeed = 0.0;
    private ElapsedTime gateTimer;
    private ElapsedTime gripTimer;
    private boolean relicArmDown = false;

    private TeleopEnum.grabberPosition curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
    private TeleopEnum.boxPosition curBoxPosition = TeleopEnum.boxPosition.STOWED;
    private TeleopEnum.glyphGate curGlyphGate = TeleopEnum.glyphGate.LOWERED;

    private boolean relicBeakClosed = true;
    private double speed = 0.75;
    private double glyphStopperWait = 500;
    private TeleopEnum.driveMode curDriveMode = TeleopEnum.driveMode.HALO_DRIVE;
    private TeleopEnum.liftHeight curLiftHeight = TeleopEnum.liftHeight.GROUND;

    @Override
    public void initialize() {

        robot = new HDRobot(hardwareMap);
        driverGamepad = new HDGamepad(gamepad1, this);
        servoBoyGamepad = new HDGamepad(gamepad2, this);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        gateTimer = new ElapsedTime();
        gripTimer = new ElapsedTime();
        driverGamepad.setGamepad(gamepad1);
        servoBoyGamepad.setGamepad(gamepad2);
    }


    @Override
    public void continuousRun(double elapsedTime) {
        if(robot.IMU1.isCalibrated()) {
            telemetry();
            driveTrain();
            glyph();
            relic();
        }else{
            robot.robotDrive.motorBreak();
        }
    }

    private void relic(){
        if(relicArmDown){
            robot.robotRelic.setRelicArmDown(gamepad2.left_stick_y/30);
        }
        if(relicBeakClosed){
            robot.robotRelic.setRelicBeakClosed();
        }else{
            robot.robotRelic.setRelicBeakOpen();
        }
        robot.robotRelic.setLiftMotorPower(-gamepad2.right_stick_y);
    }

    private void glyph(){
        if(gateTimer.milliseconds() > glyphStopperWait){
            curGlyphGate = TeleopEnum.glyphGate.LOWERED;
        }

        switch (curBoxPosition) {
            case STOWED:
                robot.robotGlyph.stowBox();
                gripTimer.reset();
                break;
            case FLAT:
                curGlyphGate = TeleopEnum.glyphGate.RAISED;
                robot.robotGlyph.flatBox();
                gateTimer.reset();
                gripTimer.reset();
                break;
            case OUT:
                gateTimer.reset();
                curGlyphGate = TeleopEnum.glyphGate.RAISED;
                break;
        }

        if(gripTimer.milliseconds() > 500){
            robot.robotGlyph.extendBox();
        }

        switch (curGrabberPosition) {
            case OPEN:
                robot.robotGlyph.openBox();
                break;
            case CLOSED:
                robot.robotGlyph.gripBox();
                break;
        }

        switch (curGlyphGate) {
            case RAISED:
                robot.robotGlyph.raiseLiftGate();
                break;
            case LOWERED:
                robot.robotGlyph.lowerGlyphGate();
                break;
        }

        switch (curLiftHeight) {
            case GROUND:
                if(robot.liftMotor.getCurrentPosition() > 500){
                    robot.robotGlyph.setLiftPower(-1.0);
                }else if(robot.liftMotor.getCurrentPosition() > 300){
                    robot.robotGlyph.setLiftPower(-0.25);
                }else if(robot.liftMotor.getCurrentPosition() > 10){
                    robot.robotGlyph.setLiftPower(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower(0.0);
                    glyphStopperWait = 500;
                }
                break;
            case HIGH:
                glyphStopperWait = 2000;
                if(robot.liftMotor.getCurrentPosition() > 1900){
                    robot.robotGlyph.setLiftPower(0.0);
                }else if(robot.liftTouch.getState()){
                    robot.robotGlyph.setLiftPower(1.0);
                }else{
                    robot.robotGlyph.setLiftPower(0.0);
                }
                break;
        }
    }

    private void telemetry(){
        dashboard.addProgramSpecificTelemetry(1, "Speed: " + String.valueOf(speed));
        dashboard.addProgramSpecificTelemetry(2, "Drive Mode: %s", String.valueOf(curDriveMode));
        dashboard.addProgramSpecificTelemetry(3, "LF: %s, RF: %s, LB: %s, RB: %s", String.valueOf(robot.frontLeft.getCurrentPosition())
        , String.valueOf(robot.frontRight.getCurrentPosition()), String.valueOf(robot.backLeft.getCurrentPosition()), String.valueOf(robot.backRight.getCurrentPosition()));
        dashboard.addProgramSpecificTelemetry(4, "Lift Encoder: %s", String.valueOf(robot.liftMotor.getCurrentPosition()));
        dashboard.addProgramSpecificTelemetry(5, "Touch Sensor: %s", String.valueOf(robot.liftTouch.getState()));
    }

    private void driveTrain() {
        switch (curDriveMode) {
            case FIELD_CENTRIC_DRIVE:
                double gyroHeading = robot.IMU1.getZheading();
                dashboard.addDiagnosticSpecificTelemetry(0, "Gyro Z Heading: %f", gyroHeading);
                robot.robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x * speed, gamepad1.left_stick_y * speed, gamepad1.right_stick_x * speed, gyroHeading);
                break;
            case HALO_DRIVE:
                robot.robotDrive.haloDrive(gamepad1.left_stick_x * speed, gamepad1.left_stick_y * speed, gamepad1.right_stick_x * speed);
                break;
            case TANK_DRIVE:
                robot.robotDrive.tankDrive(gamepad1.left_stick_y * speed, gamepad1.right_stick_y * speed);
                break;
        }
    }


    @Override
    public void buttonChange(HDGamepad instance, HDGamepad.gamepadButtonChange button, boolean pressed) {
        if(instance == driverGamepad){
            switch (button) {
                case A:
                    if(pressed){
                        curLiftHeight = TeleopEnum.liftHeight.GROUND;
                        curBoxPosition = TeleopEnum.boxPosition.STOWED;
                        curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                    }
                    break;
                case B:
                    break;
                case X:
                    if(pressed && curGrabberPosition == TeleopEnum.grabberPosition.OPEN){
                        robot.robotGlyph.setIntakePower(-1.0);
                        robot.robotGlyph.backwardsConveyor();
                    }else{
                        robot.robotGlyph.setIntakePower(0.0);
                        robot.robotGlyph.stopConveyor();
                    }
                    break;
                case Y:
                    if(pressed && curBoxPosition == TeleopEnum.boxPosition.OUT){
                        curLiftHeight = TeleopEnum.liftHeight.HIGH;
                    }
                    break;
                case DPAD_LEFT:
                    if(pressed){
                        curDriveMode = curDriveMode.getNext();
                    }
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    if(pressed) {
                        speed += 0.25;
                        speed = Range.clip(speed, 0.25, 1.0);
                    }
                    break;
                case DPAD_DOWN:
                    if(pressed) {
                        speed -= 0.25;
                        speed = Range.clip(speed, 0.25, 1.0);
                    }
                    break;
                case LEFT_BUMPER:
                    if(pressed){
                        switch (curBoxPosition) {
                            case STOWED:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                curBoxPosition = TeleopEnum.boxPosition.OUT;
                                break;
                            case FLAT:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                curBoxPosition = TeleopEnum.boxPosition.OUT;
                                break;
                            case OUT:
                                if(curLiftHeight != TeleopEnum.liftHeight.HIGH) {
                                    curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                                    curBoxPosition = TeleopEnum.boxPosition.STOWED;
                                }
                                break;
                        }
                    }
                    break;
                case RIGHT_BUMPER:
                    if(pressed && curGrabberPosition == TeleopEnum.grabberPosition.OPEN){
                        robot.robotGlyph.setIntakePower(1.0);
                        robot.robotGlyph.startConveyor();
                    }else{
                        robot.robotGlyph.setIntakePower(0.0);
                        robot.robotGlyph.stopConveyor();
                    }
                    break;
                case RIGHT_TRIGGER:
                    if(pressed){
                        lastSpeed = speed;
                        speed = .25;
                    }else{
                        speed = lastSpeed;
                    }
                    break;
                case LEFT_TRIGGER:
                    if(pressed){
                        switch (curGrabberPosition) {
                            case OPEN:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                break;
                            case CLOSED:
                                curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                                break;
                        }
                    }
                    break;
                case START:
                    if(pressed){
                        robot.IMU1.zeroZheading();
                    }
                    break;
            }
        }else if(instance == servoBoyGamepad){
            switch (button) {
                case A:
                    if(pressed){
                        curLiftHeight = TeleopEnum.liftHeight.GROUND;
                        curBoxPosition = TeleopEnum.boxPosition.STOWED;
                        curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                    }
                    break;
                case B:
                    break;
                case X:
                    if(pressed && curGrabberPosition == TeleopEnum.grabberPosition.OPEN){
                        robot.robotGlyph.setIntakePower(-1.0);
                        robot.robotGlyph.backwardsConveyor();
                    }else{
                        robot.robotGlyph.setIntakePower(0.0);
                        robot.robotGlyph.stopConveyor();
                    }
                    break;
                case Y:
                    if(pressed && curBoxPosition == TeleopEnum.boxPosition.OUT){
                        curLiftHeight = TeleopEnum.liftHeight.HIGH;
                    }
                    break;
                case DPAD_LEFT:
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    if(pressed){
                        relicArmDown = !relicArmDown;
                        if(relicArmDown){
                            robot.robotRelic.setRelicArmDown(0.0);
                        }else{
                            robot.robotRelic.setRelicArmUp();
                        }
                    }
                    break;
                case DPAD_DOWN:
                    break;
                case LEFT_BUMPER:
                    if(pressed){
                        switch (curBoxPosition) {
                            case STOWED:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                curBoxPosition = TeleopEnum.boxPosition.OUT;
                                break;
                            case FLAT:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                curBoxPosition = TeleopEnum.boxPosition.OUT;
                                break;
                            case OUT:
                                if(curLiftHeight != TeleopEnum.liftHeight.HIGH) {
                                    curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                                    curBoxPosition = TeleopEnum.boxPosition.STOWED;
                                }
                                break;
                        }
                    }
                    break;
                case RIGHT_BUMPER:
                    if(pressed && curGrabberPosition == TeleopEnum.grabberPosition.OPEN){
                        robot.robotGlyph.setIntakePower(1.0);
                        robot.robotGlyph.startConveyor();
                    }else{
                        robot.robotGlyph.setIntakePower(0.0);
                        robot.robotGlyph.stopConveyor();
                    }
                    break;
                case RIGHT_TRIGGER:
                    if(pressed && relicArmDown){
                        relicBeakClosed = !relicBeakClosed;
                    }
                    break;
                case LEFT_TRIGGER:
                    if(pressed){
                        switch (curGrabberPosition) {
                            case OPEN:
                                curGrabberPosition = TeleopEnum.grabberPosition.CLOSED;
                                break;
                            case CLOSED:
                                curGrabberPosition = TeleopEnum.grabberPosition.OPEN;
                                break;
                        }
                    }
                    break;
                case START:
                    break;
            }
        }
    }
}
