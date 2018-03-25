package org.firstinspires.ftc.hdcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.hdlib.Controls.HDGamepad;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.HDRobot;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.MecanumPathPlanner;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;

import java.util.Arrays;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class PathPlanTest extends HDOpMode{

    private HDRobot robot;

    double[][] waypoints = new double[][]{
            {0, 1, 0},
            {0, 2, 0},
    };
    double totalTime = 2; //seconds
    double timeStep = 0.1; //period of control loop on Rio, seconds
    int stepTrack = 0;

    ElapsedTime timeTrack = new ElapsedTime();

    MecanumPathPlanner path;

    @Override
    public void initialize() {

        robot = new HDRobot(hardwareMap);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        double robotTrackWidth = 1.2816667; //distance between left and right wheels, feet (was 2)
        double robotTrackLength = 1.058333; //distance between front and rear wheels, feet was 2.5

        path = new MecanumPathPlanner(waypoints);
        path.setPathAlpha(0.9);
        path.setPathBeta(0.5);
        path.calculate(totalTime, timeStep, robotTrackWidth, robotTrackLength);

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        timeTrack.reset();
    }


    @Override
    public void continuousRun(double elapsedTime) {
        if(robot.IMU1.isCalibrated()) {
            if(timeTrack.milliseconds() > 100){
                stepTrack = stepTrack + 1;
                timeTrack.reset();
            }
            try{
                robot.robotDrive.setMotorSpeeds((new double[]{path.smoothLeftFrontVelocity[stepTrack][1], path.smoothRightFrontVelocity[stepTrack][1],
                        path.smoothLeftRearVelocity[stepTrack][1], path.smoothRightRearVelocity[stepTrack][1]}));
            }catch(Exception e){
                robot.robotDrive.setMotorSpeeds((new double[]{0, 0, 0, 0}));
            }
        }else{
            robot.robotDrive.motorBreak();
        }
    }

}
