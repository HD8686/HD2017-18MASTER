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
import org.firstinspires.ftc.hdlib.RobotHardwareLib.StandardPathPlanner;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;

/**
 * Created by akash on 8/8/2017.
 */

@TeleOp
public class PathPlanTest extends HDOpMode{

    private HDRobot robot;

    double[][] waypoints = new double[][]{
            {1/5.0, 1/5.0},
            {5/5.0, 1/5.0},
            {9/5.0, 12/5.0},
    };
    double totalTime = 5.5; //seconds
    double timeStep = 0.1; //period of control loop on Rio, seconds
    int stepTrack = 0;

    ElapsedTime timeTrack = new ElapsedTime();

    StandardPathPlanner path;

    @Override
    public void initialize() {

        robot = new HDRobot(hardwareMap);

        robot.robotDrive.reverseSide(HDDriveHandler.Side.Right);
        robot.robotDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        double robotTrackWidth = 1.2816667; //distance between left and right wheels, feet (was 2)
        double robotTrackLength = 1.058333; //distance between front and rear wheels, feet was 2.5

        path = new StandardPathPlanner(waypoints);
        path.setPathAlpha(0.9);
        path.setPathBeta(0.5);
        path.calculate(totalTime, timeStep, robotTrackWidth);

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
        //if(robot.IMU1.isCalibrated()) {
            if(timeTrack.milliseconds() > 100){
                stepTrack = stepTrack + 1;
                timeTrack.reset();
            }
            try{
                double gyroAdjust = updateHeadingController();
                Log.w("leftVelocity", String.valueOf(path.smoothLeftVelocity[stepTrack][1]));
                Log.w("rightVelocity", String.valueOf(path.smoothRightVelocity[stepTrack][1]));
                //rob   ot.robotDrive.setMotorSpeeds((new double[]{path.smoothLeftVelocity[stepTrack][1] + gyroAdjust, path.smoothRightVelocity[stepTrack][1] - gyroAdjust,
                //        path.smoothLeftVelocity[stepTrack][1] + gyroAdjust, path.smoothRightVelocity[stepTrack][1] - gyroAdjust}));
                Log.w("Heading", String.valueOf(path.heading[stepTrack][1]));
            }catch(Exception e){
                robot.robotDrive.setMotorSpeeds((new double[]{0, 0, 0, 0}));
            }
        //}else{
         //   robot.robotDrive.motorBreak();
        //}
    }

    double updateHeadingController() {
        //return ((path.heading[stepTrack][1] - robot.IMU1.getZheading()) * .005);
        return 0;
    }

}
