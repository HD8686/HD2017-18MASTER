package org.firstinspires.ftc.hdlib.OpModeManagement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.hdlib.Telemetry.HDFormatter;
import org.firstinspires.ftc.hdlib.Telemetry.HDTelemetry;
import org.firstinspires.ftc.hdlib.Telemetry.HDDashboard;

/**
 * Created by akash on 8/4/2017.
 */

public abstract class HDOpMode extends LinearOpMode {

    private HDTelemetry mDisplay;
    public HDDashboard dashboard;
    public HDFormatter formatter;
    public ElapsedTime elapsedTime = new ElapsedTime();
    private static HDOpMode instance = null;
    private HDLoopInterface hdLoopInterface;

    public HDOpMode(){
        super();
        instance = this;
    }

    public static HDOpMode getInstance() {return instance; }

    public abstract void initialize();
    public abstract void initializeLoop();
    public abstract void Start();
    public abstract void continuousRun(double elapsedTime);

    @Override
    public void runOpMode() throws InterruptedException {
        mDisplay = new HDTelemetry(telemetry);
        hdLoopInterface = new HDLoopInterface();
        dashboard = new HDDashboard(mDisplay);
        this.formatter = new HDFormatter();

        initialize();
        hdLoopInterface.runInitializeLoopInterface();

        while(!opModeIsActive() && !isStopRequested()){
            hdLoopInterface.runInitializeLoopInterface();
            initializeLoop();
        }

        waitForStart();
        elapsedTime.reset();
        hdLoopInterface.runStartInterface();
        Start();

        while(opModeIsActive() && !isStopRequested()){
            continuousRun(elapsedTime.seconds());
            hdLoopInterface.runContinuousRunInterface();
        }
    }

}
