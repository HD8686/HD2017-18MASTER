package org.firstinspires.ftc.hdcode.Autonomous;

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

/**
 * Created by FIRSTMentor on 1/13/2018.
 */

public class Relic_DoubleGlyph_Jewel implements HDAuto{

    private enum States{
        delay,
        scanVuMark,
        lowerJewelArm,
        readJewel,
        hitJewel,
        turnBack,
        driveOffBoard,
        straightenUp,
        driveToDistance,
    }

    private HDRobot robot;
    private HDStateMachine SM;
    private HDVuforiaVuMarks vuforiaVuMarks;
    RelicRecoveryVuMark vuMark;
    ElapsedTime failsafeTimer;

    private HDDashboard dashboard;
    private double delay;
    private Alliance alliance;
    private boolean turnLeft;

    public Relic_DoubleGlyph_Jewel(double delay, Alliance alliance, HardwareMap hardwareMap, HDDashboard dashboard){

        this.dashboard = dashboard;

        robot = new HDRobot(hardwareMap);

        SM = new HDStateMachine(robot.robotDrive);

        vuforiaVuMarks = new HDVuforiaVuMarks(hardwareMap);
        vuforiaVuMarks.activateTracking();

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.resetEncoders();
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.robotDrive.setAlliance(alliance);

        robot.robotJewel.raiseLeftServo();

        this.delay = delay;
        this.alliance = alliance;

        failsafeTimer = new ElapsedTime();

        Runnable reset = new Runnable() {
            @Override
            public void run() {
                robot.robotDrive.resetPIDvalues();
            }
        };

        SM.setResetCode(reset);
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
                case scanVuMark:
                    SM.setNextState(States.lowerJewelArm, HDWaitTypes.Timer, 6.0);
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
                    robot.robotGlyph.setLiftPower(0);
                    if(alliance == Alliance.RED_ALLIANCE){
                        robot.robotJewel.lowerLeftServo();
                        if(robot.robotJewel.getLeftColor() != HDJewel.jewelColor.INCONCLUSIVE){
                            SM.setNextState(States.readJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }else{
                        robot.robotJewel.lowerLeftServo();
                        if(robot.robotJewel.getLeftColor() != HDJewel.jewelColor.INCONCLUSIVE){
                            SM.setNextState(States.readJewel, HDWaitTypes.Timer, 0.05);
                        }
                    }
                    break;
                case readJewel:
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
                    if(turnLeft){
                        SM.setNextState(States.turnBack, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(-12, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }else{
                        SM.setNextState(States.turnBack, HDWaitTypes.driveHandlerTarget);
                        robot.robotDrive.gyroTurn(12, 0.015, 0.000004, 0.0006, 0.0, 1.0, 1.0, -1.0, robot.IMU1.getZheading());
                    }
                    break;
                case turnBack:
                    SM.setNextState(States.driveOffBoard, HDWaitTypes.driveHandlerTarget);
                    robot.robotJewel.raiseLeftServo();
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveOffBoard:
                    SM.setNextState(States.straightenUp, HDWaitTypes.EncoderChangeBoth, 1000.0);
                    if(alliance == Alliance.RED_ALLIANCE)
                        robot.robotDrive.tankDrive(-.25, -.25);
                    else
                        robot.robotDrive.tankDrive(.25, .25);
                    break;
                case straightenUp:
                    SM.setNextState(States.driveToDistance, HDWaitTypes.driveHandlerTarget);
                    robot.robotDrive.gyroTurn(0, 0.015, 0.000004, 0.0006, 0.0, 2.0, 1.0, -1.0, robot.IMU1.getZheading());
                    break;
                case driveToDistance:
                    if(alliance == Alliance.RED_ALLIANCE){
                        robot.robotDrive.tankDrive(-.25, -.25);
                        if(robot.frontUS.getDistanceCM() > 25){
                            //CHANGE NUMBER
                        }
                    }else{
                        robot.robotDrive.tankDrive(.25, .25);
                        if(robot.backUS.getDistanceCM() > 25){
                            //CHANGE NUMBER
                        }
                    }

                    break;
            }
        }
    }

    private boolean ready(){
        boolean ready = true;
        if(!robot.IMU1.isCalibrated()){
            ready = false;
        }
        return ready;
    }

}
