package org.firstinspires.ftc.hdcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class HDGlyphTesting extends HDOpMode implements HDGamepad.HDButtonMonitor{

    private HDGamepad driverGamepad;
    private HDGamepad servoBoyGamepad;
    private HDRobot robot;
    private double gripperCalibrator = 0;
    private double kickerCalibrator = 0;

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
            dashboard.addProgramSpecificTelemetry(0, "Gyro Z Heading: %f", robot.IMU1.getZheading());
            dashboard.addProgramSpecificTelemetry(1, "Left Color: %s, Right Color: %s", String.valueOf(robot.bottomLeftColor.red()), String.valueOf(robot.bottomRightColor.red()));
            dashboard.addProgramSpecificTelemetry(2, "Left Distance: %s, Right Distance: %s", String.valueOf(robot.robotJewel.leftDistance.getDistance(DistanceUnit.CM)), String.valueOf(robot.robotJewel.rightDistance.getDistance(DistanceUnit.CM)));
            dashboard.addProgramSpecificTelemetry(3, "Left enc: %s, Right enc: %s", String.valueOf(robot.robotGlyph.leftPinionMotor.getCurrentPosition()), String.valueOf(robot.robotGlyph.rightPinionMotor.getCurrentPosition()));
            dashboard.addProgramSpecificTelemetry(4, "Scotch Enc: %s", String.valueOf(robot.robotGlyph.scotchYokeMotor.getCurrentPosition()));
            dashboard.addProgramSpecificTelemetry(5, "Value: %s", String.valueOf(kickerCalibrator));
            robot.robotDrive.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);
        }else{
            robot.robotDrive.motorBreak();
        }
    }

    @Override
    public void buttonChange(HDGamepad instance, HDGamepad.gamepadButtonChange button, boolean pressed) {
        if(instance == driverGamepad){
            switch (button) {
                case A:
                    if(pressed){
                        robot.robotGlyph.scotchYokeMotor.setTargetPosition(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() + 1500);
                    }
                    break;
                case B:
                    if(pressed){
                        robot.robotGlyph.leftPinionMotor.setTargetPosition(10000);
                        robot.robotGlyph.rightPinionMotor.setTargetPosition(10000);
                    }
                    break;
                case X:
                    if(pressed){
                        robot.robotGlyph.bottomLeftIntake.setPower(-.65);
                        robot.robotGlyph.bottomRightIntake.setPower(.65);
                        robot.robotGlyph.topLeftIntake.setPower(-.65);
                        robot.robotGlyph.topRightIntake.setPower(.65);
                    }else{
                        robot.robotGlyph.bottomLeftIntake.setPower(0.0);
                        robot.robotGlyph.bottomRightIntake.setPower(0.0);
                        robot.robotGlyph.topLeftIntake.setPower(0.0);
                        robot.robotGlyph.topRightIntake.setPower(0.0);
                    }
                    break;
                case Y:
                    if(pressed){
                        robot.robotGlyph.leftPinionMotor.setTargetPosition(0);
                        robot.robotGlyph.rightPinionMotor.setTargetPosition(0);
                    }
                    break;
                case DPAD_LEFT:
                    if(pressed){
                        robot.robotGlyph.bottomLeftIntake.setPower(.75);
                        robot.robotGlyph.bottomRightIntake.setPower(-.75);
                        robot.robotGlyph.topLeftIntake.setPower(.75);
                        robot.robotGlyph.topRightIntake.setPower(-.75);
                    }else{
                        robot.robotGlyph.bottomLeftIntake.setPower(0.0);
                        robot.robotGlyph.bottomRightIntake.setPower(0.0);
                        robot.robotGlyph.topLeftIntake.setPower(0.0);
                        robot.robotGlyph.topRightIntake.setPower(0.0);
                    }
                    break;
                case DPAD_RIGHT:
                    if(pressed){
                        robot.robotGlyph.leftBlockGrabber.setPosition(0.80);
                        robot.robotGlyph.rightBlockGrabber.setPosition(0.36);
                    }else{
                        robot.robotGlyph.leftBlockGrabber.setPosition(0.92);
                        robot.robotGlyph.rightBlockGrabber.setPosition(0.27);
                    }
                    break;
                case DPAD_UP:
                    if(pressed) {
                        robot.robotGlyph.blockKicker.setPosition(0.12);
                    }else{
                        robot.robotGlyph.blockKicker.setPosition(0.06);
                    }
                    break;
                case DPAD_DOWN:
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
                    break;
                case Y:
                    break;
                case DPAD_LEFT:
                    break;
                case DPAD_RIGHT:
                    break;
                case DPAD_UP:
                    break;
                case DPAD_DOWN:
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
        }
    }
}