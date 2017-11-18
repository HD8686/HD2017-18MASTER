package org.firstinspires.ftc.hdcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
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

    private boolean collectorOn = false;
    private boolean gripBlock = false;

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

    @Override
    public void initialize() {
        robot = new HDRobot(hardwareMap);

        driverGamepad = new HDGamepad(gamepad1, this);
        servoBoyGamepad = new HDGamepad(gamepad2, this);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.robotJewel.raiseLeftServo();
        robot.robotJewel.raiseRightServo();
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
        }else{
            robot.robotDrive.motorBreak();
        }


    }

    private void telemetry(){
        dashboard.addProgramSpecificTelemetry(0, "Speed: " + String.valueOf(speed));
        dashboard.addProgramSpecificTelemetry(1, "Drive Mode: %s", String.valueOf(curDriveMode));
        dashboard.addProgramSpecificTelemetry(2, "Collector On?: %s", String.valueOf(collectorOn));
        dashboard.addDiagnosticSpecificTelemetry(0, "Gyro Z Heading: %f", robot.IMU1.getZheading());
    }

    private void driveTrain(){
        switch (curDriveMode) {
            case FIELD_CENTRIC_DRIVE:
                robot.robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x*speed, gamepad1.left_stick_y*speed, gamepad1.right_stick_x*speed, robot.IMU1.getZheading());
                break;
            case HALO_DRIVE:
                robot.robotDrive.haloDrive(gamepad1.left_stick_x*speed, gamepad1.left_stick_y*speed, gamepad1.right_stick_x*speed);
                break;
            case TANK_DRIVE:
                robot.robotDrive.tankDrive(gamepad1.left_stick_y*speed, gamepad1.right_stick_y*speed);
                break;
        }
    }

    private void glyphSystem(){
    if(gamepad2.right_bumper){
        robot.robotGlyph.setLiftPower(gamepad2.left_stick_y);
    }else if(gamepad2.start){
        robot.robotGlyph.leftPinionMotor.setPower(gamepad2.left_stick_y);
        robot.robotGlyph.rightPinionMotor.setPower(gamepad2.right_stick_y);
    }
    if(gamepad2.y){
        gripBlock = false;
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
                    break;
                case LEFT_TRIGGER:
                    break;
                case START:
                    break;
            }
        }else if(instance == servoBoyGamepad){
            switch (button) {
                case A:
                    break;
                case B:
                    break;
                case X:
                    if(pressed) {
                        collectorOn = !collectorOn;
                    }
                    break;
                case Y:
                    break;
                case DPAD_LEFT:
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    if(pressed){
                        robot.robotGlyph.scotchYokeTop();
                    }
                    break;
                case DPAD_DOWN:
                    if(pressed){
                        robot.robotGlyph.scotchYokeBottom();
                    }
                    break;
                case LEFT_BUMPER:
                    if(pressed){
                        robot.robotGlyph.gripBlock();
                    }else{
                        robot.robotGlyph.unGripBlock();
                    }
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    break;
                case LEFT_TRIGGER:
                    break;
                case START:

                    break;
            }
        }
    }
}
