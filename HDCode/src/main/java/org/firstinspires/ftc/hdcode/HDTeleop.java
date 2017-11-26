package org.firstinspires.ftc.hdcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class HDTeleop extends HDOpMode implements HDGamepad.HDButtonMonitor{

    private HDGamepad driverGamepad;
    private HDGamepad servoBoyGamepad;
    private HDRobot robot;

    private boolean collectorOn = false;

    private double lastSpeed = 0.0;

    private ElapsedTime loopTime;

    private enum liftHeight{
        GROUND, //0
        COLLECT2, //1000
        DEPOSITHIGH, //9800
        BALANCINGSTONE, //2000
    }

    private enum scotchYokePosition{
        UP,
        DOWN,
        BRING_BLOCK_UP,
        NORMAL,
        GRAB_BOTTOM_BLOCK
    }

    private enum driveMode{
        FIELD_CENTRIC_DRIVE,
        HALO_DRIVE,
        TANK_DRIVE;

        public driveMode getNext() {
            return this.ordinal() < driveMode.values().length - 1
                    ? driveMode.values()[this.ordinal() + 1]
                    : driveMode.values()[0];
        }
    }

    private double speed = 0.75;
    private driveMode curDriveMode = driveMode.HALO_DRIVE;
    private liftHeight curLiftHeight = liftHeight.GROUND;
    private scotchYokePosition scotchPos = scotchYokePosition.DOWN;

    @Override
    public void initialize() {
        robot = new HDRobot(hardwareMap);

        driverGamepad = new HDGamepad(gamepad1, this);
        servoBoyGamepad = new HDGamepad(gamepad2, this);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.robotJewel.raiseLeftServo();
        robot.robotJewel.raiseRightServo();

        robot.robotGlyph.resetLiftEncoders();
        robot.robotGlyph.resetScotchYoke();

        loopTime = new ElapsedTime();
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        driverGamepad.setGamepad(gamepad1);
        servoBoyGamepad.setGamepad(gamepad2);
    }

    @Override
    public void continuousRun(double elapsedTime) {
        if(robot.IMU1.isCalibrated()) {
            telemetry();
            driveTrain();
            glyphSystem();
            dashboard.addDiagnosticSpecificTelemetry(0, "Loop Time: %s MS", formatter.formatDecimals(loopTime.milliseconds(), 2));
            loopTime.reset();
        }else{
            robot.robotDrive.motorBreak();
        }
    }

    private void telemetry(){
        dashboard.addProgramSpecificTelemetry(0, "Speed: " + String.valueOf(speed));
        dashboard.addProgramSpecificTelemetry(1, "Drive Mode: %s", String.valueOf(curDriveMode));
        dashboard.addProgramSpecificTelemetry(2, "Collector On?: %s", String.valueOf(collectorOn));
        dashboard.addDiagnosticSpecificTelemetry(1, "Lift Encoder Value: %d", (robot.robotGlyph.leftPinionMotor.getCurrentPosition()+robot.robotGlyph.rightPinionMotor.getCurrentPosition())/2);
        dashboard.addProgramSpecificTelemetry(3, "Scotch Mode: %s", String.valueOf(scotchPos));
        dashboard.addProgramSpecificTelemetry(4, "Lift Mode: %s", String.valueOf(curLiftHeight));
        /*if(robot.robotGlyph.bottomGlyphDistance.getDistance(DistanceUnit.INCH) < 2.5){
            dashboard.addDiagnosticSpecificTelemetry(3, "Block in bottom bay");
        }else{
            dashboard.addDiagnosticSpecificTelemetry(3, "No Block in bottom bay");
        }*/

    }

    private void driveTrain(){
        if(gamepad1.a){
            robot.robotDrive.gyroTurn(90.0, 0.018, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
        }else if(gamepad1.b){
            robot.robotDrive.gyroTurn(-90.0, 0.018, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
        }else if(gamepad1.y){
            robot.robotDrive.gyroTurn(0, 0.018, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
        }else {
            switch (curDriveMode) {
                case FIELD_CENTRIC_DRIVE:
                    double gyroHeading = robot.IMU1.getZheading();
                    dashboard.addDiagnosticSpecificTelemetry(0, "Gyro Z Heading: %f",gyroHeading);
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
    }

    private void glyphSystem(){


    switch (scotchPos) {
        case UP:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1900){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1600){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1200){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1500){
                robot.robotGlyph.scotchYokeMotor.setPower(.25);
            }else{
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
        case DOWN:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 400){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 50){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -400){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -50){
                robot.robotGlyph.scotchYokeMotor.setPower(0.25);
            }else{
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
        case BRING_BLOCK_UP:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 400){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 50){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -400){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -50){
                robot.robotGlyph.scotchYokeMotor.setPower(0.25);
                robot.robotGlyph.unGripBlock();
            }else{
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
                robot.robotGlyph.gripBlock();
                curLiftHeight = liftHeight.COLLECT2;
                scotchPos = scotchYokePosition.UP;
            }
            break;
        case NORMAL:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 400){
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 50){
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -400){
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -50){
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(0.25);
            }else{
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
        case GRAB_BOTTOM_BLOCK:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 400){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 50){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -400){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < -50){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(0.25);
            }else{
                if(((robot.robotGlyph.leftPinionMotor.getCurrentPosition()+robot.robotGlyph.rightPinionMotor.getCurrentPosition())/2) < 20) {
                    robot.robotGlyph.gripBlock();
                }
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
    }

    if(gamepad2.right_bumper){
        robot.robotGlyph.setLiftPower(-gamepad2.left_stick_y);
    }else if(gamepad2.start){
        robot.robotGlyph.leftPinionMotor.setPower(-gamepad2.left_stick_y);
        robot.robotGlyph.rightPinionMotor.setPower(-gamepad2.right_stick_y);
    }else{
        switch (curLiftHeight) {
            case GROUND:
                if(robot.robotGlyph.getLiftHeight() > 2000){
                    robot.robotGlyph.setLiftPower(-1);
                }else if(robot.robotGlyph.getLiftHeight() > 1000){
                    robot.robotGlyph.setLiftPower(-0.75);
                }else if(robot.robotGlyph.getLiftHeight() > 500){
                    robot.robotGlyph.setLiftPower(-0.5);
                }else if(robot.robotGlyph.getLiftHeight() > 200){
                    robot.robotGlyph.setLiftPower(-0.25);
                }
                else if(robot.robotGlyph.getLiftHeight() < 20){
                    robot.robotGlyph.setLiftPower(0);
                }
                break;
            case COLLECT2:
                if(robot.robotGlyph.getLiftHeight() < 750){
                    robot.robotGlyph.setLiftPower(0.5);
                }else if(robot.robotGlyph.getLiftHeight() < 1000){
                    robot.robotGlyph.setLiftPower(.15);
                }else if(robot.robotGlyph.getLiftHeight() > 2000){
                    robot.robotGlyph.setLiftPower(-1.0);
                }else if(robot.robotGlyph.getLiftHeight() > 1400){
                    robot.robotGlyph.setLiftPower(-.75);
                }else if(robot.robotGlyph.getLiftHeight() > 1200){
                    robot.robotGlyph.setLiftPower(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower(0.0);
                }
                break;
            case DEPOSITHIGH:
                if(robot.robotGlyph.getLiftHeight() < 8000){
                    robot.robotGlyph.setLiftPower(1.0);
                }else if(robot.robotGlyph.getLiftHeight() < 9000){
                    robot.robotGlyph.setLiftPower(.5);
                }else if(robot.robotGlyph.getLiftHeight() < 9700){
                    robot.robotGlyph.setLiftPower(0.25);
                }else if(robot.robotGlyph.getLiftHeight() > 9800){
                    robot.robotGlyph.setLiftPower(-.15);
                }else{
                    robot.robotGlyph.setLiftPower(0.0);
                }
                break;
            case BALANCINGSTONE:
                if(robot.robotGlyph.getLiftHeight() < 1750){
                    robot.robotGlyph.setLiftPower(0.75);
                }else if(robot.robotGlyph.getLiftHeight() < 2000){
                    robot.robotGlyph.setLiftPower(.15);
                }else if(robot.robotGlyph.getLiftHeight() > 3000){
                    robot.robotGlyph.setLiftPower(-1.0);
                }else if(robot.robotGlyph.getLiftHeight() > 2400){
                    robot.robotGlyph.setLiftPower(-.75);
                }else if(robot.robotGlyph.getLiftHeight() > 2200){
                    robot.robotGlyph.setLiftPower(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower(0.0);
                }
                break;
        }
    }

    if(gamepad2.left_trigger > .25){
        scotchPos = scotchYokePosition.NORMAL;
        robot.robotGlyph.setIntakePower(-.7);
        robot.robotGlyph.blockKickerOut();
        collectorOn = false;
    }else if(collectorOn){
        robot.robotGlyph.setIntakePower(.7);
        robot.robotGlyph.blockKickerIn();
    }else{
        robot.robotGlyph.setIntakePower(0.0);
        robot.robotGlyph.blockKickerIn();
    }
    }


    @Override
    public void buttonChange(HDGamepad instance, HDGamepad.gamepadButtonChange button, boolean pressed) {
        if(instance == driverGamepad){
            switch (button) {
                case A:
                    break;
                case B:
                    break;
                case X:
                    break;
                case Y:
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
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    if(pressed){
                        lastSpeed = speed;
                        speed = .5;
                    }else{
                        speed = lastSpeed;
                    }
                    break;
                case LEFT_TRIGGER:
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
                        scotchPos = scotchYokePosition.GRAB_BOTTOM_BLOCK;
                    }
                    break;
                case B:
                    if(pressed){
                    }
                    break;
                case X:
                    if(pressed){
                        scotchPos = scotchYokePosition.NORMAL;
                    }
                    break;
                case Y:
                    if(pressed){
                        scotchPos = scotchYokePosition.BRING_BLOCK_UP;
                    }
                    break;
                case DPAD_LEFT:
                    if(pressed){
                        curLiftHeight = liftHeight.COLLECT2;
                    }
                    break;
                case DPAD_RIGHT:
                    if(pressed){
                        curLiftHeight = liftHeight.BALANCINGSTONE;
                    }else{

                    }
                    break;
                case DPAD_UP:
                    if(pressed){
                        curLiftHeight = liftHeight.DEPOSITHIGH;
                    }else{

                    }
                    break;
                case DPAD_DOWN:
                    if(pressed){
                        curLiftHeight = liftHeight.GROUND;
                    }else{

                    }
                case LEFT_BUMPER:
                    if(pressed){

                    }else{

                    }
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    if(pressed) {
                        collectorOn = !collectorOn;
                    }
                    break;
                case LEFT_TRIGGER:
                    break;
                case START:

                    break;
            }
        }
    }
}
