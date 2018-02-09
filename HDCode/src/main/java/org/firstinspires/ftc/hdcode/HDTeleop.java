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

    private enum ServoboyMode{
        GLYPH,
        RELIC
    }

    private enum RelicTiltMode{
        STOWED,
        RAISED,
    }


    private enum liftHeight{
        GROUND, //0/2
        COLLECT2, //1000/2
        DEPOSITHIGH, //9800/2
        BALANCINGSTONE, //2000/2
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
    private scotchYokePosition scotchPos = scotchYokePosition.UP;
    private ServoboyMode servoboyMode = ServoboyMode.GLYPH;
    private RelicTiltMode relicTiltMode = RelicTiltMode.STOWED;

    @Override
    public void initialize() {

        telemetry.setMsTransmissionInterval(50);

        robot = new HDRobot(hardwareMap);

        driverGamepad = new HDGamepad(gamepad1, this);
        servoBoyGamepad = new HDGamepad(gamepad2, this);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.robotJewel.raiseLeftServo();

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
            relicSystem();
            dashboard.addDiagnosticSpecificTelemetry(0, "Loop Time: %s MS", formatter.formatDecimals(loopTime.milliseconds(), 2));
            loopTime.reset();
        }else{
            robot.robotDrive.motorBreak();
        }
    }

    private void telemetry(){
        dashboard.addProgramSpecificTelemetry(0, "GAMEPAD 2 MODE: %s", String.valueOf(servoboyMode));
        dashboard.addProgramSpecificTelemetry(1, "Speed: " + String.valueOf(speed));
        dashboard.addProgramSpecificTelemetry(2, "Drive Mode: %s", String.valueOf(curDriveMode));
        dashboard.addProgramSpecificTelemetry(3, "Collector On?: %s", String.valueOf(collectorOn));
        dashboard.addDiagnosticSpecificTelemetry(1, "Lift Enc.: T: %d L: %d R: %d", (robot.robotGlyph.getLiftHeight()), robot.robotGlyph.leftPinionMotor.getCurrentPosition(), robot.robotGlyph.rightPinionMotor.getCurrentPosition());
        dashboard.addProgramSpecificTelemetry(4, "Scotch Mode: %s", String.valueOf(scotchPos));
        dashboard.addProgramSpecificTelemetry(5, "Lift Mode: %s", String.valueOf(curLiftHeight));
        dashboard.addProgramSpecificTelemetry(6, "Lift Potentiometer: %s", String.valueOf(robot.robotRelic.getPotentiometerVoltage()));
    }

    private void relicSystem(){
        /*double error;
        switch (relicTiltMode) {
            case STOWED:
                error = robot.robotRelic.getPotentiometerVoltage() - 0.88;
                robot.robotRelic.setRelicTiltPower(Range.clip(error, -1, 1));
                break;
            case RAISED:
                error = robot.robotRelic.getPotentiometerVoltage() - 1.7125;
                if(error < 0.0){
                    error = -.3;
                }else if(error < -.1){
                    error = -.35;
                }else if(error < -.5){
                    error = -.8;
                }
                if(error > .02){
                    error = -.01;
                }
                error = gamepad2.left_stick_y;
                Log.w("relicPower", String.valueOf(error));
                robot.robotRelic.setRelicTiltPower(Range.clip(error, -1, 1));
                break;
        }*/
    }

    private void driveTrain(){
        if(gamepad1.a){
            robot.robotDrive.gyroTurn(90.0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
        }else if(gamepad1.b){
                    robot.robotDrive.gyroTurn(-90.0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
            }else if(gamepad1.y){
                robot.robotDrive.gyroTurn(0, 0.025, 0.000004, 0.0006, 0.0, 2.0, 0.5, -0.5, robot.IMU1.getZheading());
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
        case DOWN:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1900){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1400){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1200){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1300){
                robot.robotGlyph.scotchYokeMotor.setPower(.25);
            }else{
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
        case UP:
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
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1900){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1400){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1200){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1300){
                robot.robotGlyph.scotchYokeMotor.setPower(.25);
                robot.robotGlyph.unGripBlock();
            }else{
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
                robot.robotGlyph.gripBlock();
                curLiftHeight = liftHeight.COLLECT2;
                scotchPos = scotchYokePosition.UP;
            }
            break;
        case NORMAL:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1900){
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1400){
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1200){
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1300){
                robot.robotGlyph.scotchYokeMotor.setPower(.25);
                robot.robotGlyph.unGripBlock();
            }else{
                robot.robotGlyph.unGripBlock();
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
        case GRAB_BOTTOM_BLOCK:
            if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1900){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.scotchYokeMotor.setPower(-1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() > 1400){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.scotchYokeMotor.setPower(-.25);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1200){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.scotchYokeMotor.setPower(1.0);
                robot.robotGlyph.unGripBlock();
            }else if(robot.robotGlyph.scotchYokeMotor.getCurrentPosition() < 1300){
                curLiftHeight = liftHeight.GROUND;
                robot.robotGlyph.scotchYokeMotor.setPower(.25);
                robot.robotGlyph.unGripBlock();
            }else{
                if(((robot.robotGlyph.leftPinionMotor.getCurrentPosition()+robot.robotGlyph.rightPinionMotor.getCurrentPosition())/2) < 10) {
                    robot.robotGlyph.gripBlock();
                }
                robot.robotGlyph.scotchYokeMotor.setPower(0.0);
            }
            break;
    }

    if(gamepad2.right_bumper && servoboyMode == ServoboyMode.GLYPH){
        Log.w("Delta", String.valueOf(robot.robotGlyph.leftPinionMotor.getCurrentPosition() - robot.robotGlyph.rightPinionMotor.getCurrentPosition()));
        robot.robotGlyph.setLiftPower(-gamepad2.left_stick_y);
    }else if(gamepad2.start && servoboyMode == ServoboyMode.GLYPH){
        robot.robotGlyph.leftPinionMotor.setPower(-gamepad2.left_stick_y);
        robot.robotGlyph.rightPinionMotor.setPower(-gamepad2.right_stick_y);
    }else{
        if((Math.abs(robot.robotGlyph.leftPinionMotor.getCurrentPosition() - robot.robotGlyph.rightPinionMotor.getCurrentPosition()) > 1850) || robot.robotGlyph.getLiftHeight() > 10200){
            robot.robotGlyph.leftPinionMotor.setPower(0);
            robot.robotGlyph.rightPinionMotor.setPower(0);
            Log.w("AUTO STOP",  String.format("Lift Enc.: T: %d L: %d R: %d", (robot.robotGlyph.getLiftHeight()), robot.robotGlyph.leftPinionMotor.getCurrentPosition(), robot.robotGlyph.rightPinionMotor.getCurrentPosition()));
            stop();
        }
        switch (curLiftHeight) {
            case GROUND:
                if(robot.robotGlyph.getLiftHeight1() > 2000/2){
                    robot.robotGlyph.setLiftPower1(-1);
                }else if(robot.robotGlyph.getLiftHeight1() > 1000/2){
                    robot.robotGlyph.setLiftPower1(-0.75);
                }else if(robot.robotGlyph.getLiftHeight1() > 500/2){
                    robot.robotGlyph.setLiftPower1(-0.5);
                }else if(robot.robotGlyph.getLiftHeight1() > 200/2){
                    robot.robotGlyph.setLiftPower1(-0.25);
                }
                else if(robot.robotGlyph.getLiftHeight1() < 40/2){
                    robot.robotGlyph.setLiftPower1(0);
                }
                break;
            case COLLECT2:
                if(robot.robotGlyph.getLiftHeight1() < 850/2){
                    robot.robotGlyph.setLiftPower1(0.5);
                }else if(robot.robotGlyph.getLiftHeight1() < 1100/2){
                    robot.robotGlyph.setLiftPower1(.25);
                }else if(robot.robotGlyph.getLiftHeight1() > 2100/2){
                    robot.robotGlyph.setLiftPower1(-1.0);
                }else if(robot.robotGlyph.getLiftHeight1() > 1500/2){
                    robot.robotGlyph.setLiftPower1(-.75);
                }else if(robot.robotGlyph.getLiftHeight1() > 1300/2){
                    robot.robotGlyph.setLiftPower1(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower1(0.0);
                }
                break;
            case DEPOSITHIGH:
                if(robot.robotGlyph.getLiftHeight1() < 7500/2){
                    robot.robotGlyph.setLiftPower1(1.0);
                }else if(robot.robotGlyph.getLiftHeight1() < 8500/2){
                    robot.robotGlyph.setLiftPower1(0.35);
                }else if(robot.robotGlyph.getLiftHeight1() < 9200/2){
                    robot.robotGlyph.setLiftPower1(0.35);
                }else if(robot.robotGlyph.getLiftHeight1() > 9300/2){
                    robot.robotGlyph.setLiftPower1(-.15);
                }else{
                    robot.robotGlyph.setLiftPower1(0.0);
                }
                break;
            case BALANCINGSTONE:
                if(robot.robotGlyph.getLiftHeight1() < 1750/2){
                    robot.robotGlyph.setLiftPower1(0.75);
                }else if(robot.robotGlyph.getLiftHeight1() < 2000/2){
                    robot.robotGlyph.setLiftPower1(.25);
                }else if(robot.robotGlyph.getLiftHeight1() > 3000/2){
                    robot.robotGlyph.setLiftPower1(-1.0);
                }else if(robot.robotGlyph.getLiftHeight1() > 2400/2){
                    robot.robotGlyph.setLiftPower1(-.75);
                }else if(robot.robotGlyph.getLiftHeight1() > 2200/2){
                    robot.robotGlyph.setLiftPower1(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower1(0.0);
                }
                break;
        }
        switch (curLiftHeight) {
            case GROUND:
                robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(robot.robotGlyph.getLiftHeight2() > 2000/2){
                    robot.robotGlyph.setLiftPower2(-1);
                }else if(robot.robotGlyph.getLiftHeight2() > 1000/2){
                    robot.robotGlyph.setLiftPower2(-0.75);
                }else if(robot.robotGlyph.getLiftHeight2() > 500/2){
                    robot.robotGlyph.setLiftPower2(-0.5);
                }else if(robot.robotGlyph.getLiftHeight2() > 200/2){
                    robot.robotGlyph.setLiftPower2(-0.25);
                }
                else if(robot.robotGlyph.getLiftHeight2() < 40/2){
                    robot.robotGlyph.setLiftPower2(0);
                }
                break;
            case COLLECT2:
                robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(robot.robotGlyph.getLiftHeight2() < 850/2){
                    robot.robotGlyph.setLiftPower2(0.5);
                }else if(robot.robotGlyph.getLiftHeight2() < 1100/2){
                    robot.robotGlyph.setLiftPower2(.25);
                }else if(robot.robotGlyph.getLiftHeight2() > 2100/2){
                    robot.robotGlyph.setLiftPower2(-1.0);
                }else if(robot.robotGlyph.getLiftHeight2() > 1500/2){
                    robot.robotGlyph.setLiftPower2(-.75);
                }else if(robot.robotGlyph.getLiftHeight2() > 1300/2){
                    robot.robotGlyph.setLiftPower2(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower2(0.0);
                }
                break;
            case DEPOSITHIGH:
                if(robot.robotGlyph.getLiftHeight2() < 7500/2){
                    robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.robotGlyph.setLiftPower2(1.0);
                }else if(robot.robotGlyph.getLiftHeight2() < 8500/2){
                    robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.robotGlyph.setLiftPower2(0.35);
                }else if(robot.robotGlyph.getLiftHeight2() < 9200/2){
                    robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.robotGlyph.setLiftPower2(0.35);
                }else if(robot.robotGlyph.getLiftHeight2() > 9300/2){
                    robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.robotGlyph.setLiftPower2(-.15);
                }else{
                    robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.robotGlyph.setLiftPower2(0.1);
                }
                break;
            case BALANCINGSTONE:
                robot.robotGlyph.rightPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(robot.robotGlyph.getLiftHeight2() < 1750/2){
                    robot.robotGlyph.setLiftPower2(0.75);
                }else if(robot.robotGlyph.getLiftHeight2() < 2000/2){
                    robot.robotGlyph.setLiftPower2(.25);
                }else if(robot.robotGlyph.getLiftHeight2() > 3000/2){
                    robot.robotGlyph.setLiftPower2(-1.0);
                }else if(robot.robotGlyph.getLiftHeight2() > 2400/2){
                    robot.robotGlyph.setLiftPower2(-.75);
                }else if(robot.robotGlyph.getLiftHeight2() > 2200/2){
                    robot.robotGlyph.setLiftPower2(-0.15);
                }else{
                    robot.robotGlyph.setLiftPower2(0.0);
                }
                break;
        }
    }

    if((gamepad2.left_trigger > .25 && servoboyMode == ServoboyMode.GLYPH) || gamepad1.left_trigger > .25){
        scotchPos = scotchYokePosition.NORMAL;
        robot.robotGlyph.setIntakePower(-.7);
        robot.robotGlyph.blockKickerOut();
        collectorOn = false;
    }else if(collectorOn){
        if(gamepad1.left_bumper){
            robot.robotGlyph.setIntakePowerIndividual(-.7, .7, .7, .7);
            robot.robotGlyph.blockKickerIn();
        }else if(gamepad1.right_bumper){
            robot.robotGlyph.setIntakePowerIndividual(.7, -.7, .7, .7);
            robot.robotGlyph.blockKickerIn();
        }else{
            robot.robotGlyph.setIntakePower(.7);
            robot.robotGlyph.blockKickerIn();
        }
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
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        scotchPos = scotchYokePosition.GRAB_BOTTOM_BLOCK;
                    }
                    break;
                case B:
                    if(pressed){
                    }
                    break;
                case X:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        scotchPos = scotchYokePosition.NORMAL;
                    }
                    break;
                case Y:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        scotchPos = scotchYokePosition.BRING_BLOCK_UP;
                    }
                    break;
                case DPAD_LEFT:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        curLiftHeight = liftHeight.COLLECT2;
                    }
                    break;
                case DPAD_RIGHT:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        curLiftHeight = liftHeight.BALANCINGSTONE;
                    }else{

                    }
                    break;
                case DPAD_UP:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        curLiftHeight = liftHeight.DEPOSITHIGH;
                    }else{

                    }
                    break;
                case DPAD_DOWN:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH){
                        curLiftHeight = liftHeight.GROUND;
                    }else{

                    }
                case LEFT_BUMPER:
                    if(pressed){
                        relicTiltMode = RelicTiltMode.RAISED;
                    }
                    break;
                case RIGHT_BUMPER:
                    break;
                case RIGHT_TRIGGER:
                    if(pressed && servoboyMode == ServoboyMode.GLYPH) {
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
