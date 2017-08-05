package org.firstinspires.ftc.hdlib.OpModeManagement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by akash on 8/4/2017.
 */

public abstract class HDOpMode extends LinearOpMode {

    public ElapsedTime elapsedTime = new ElapsedTime();
    private static HDOpMode instance = null;

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
        initialize();

        while(!opModeIsActive() && !isStopRequested()){
            initializeLoop();
        }

        waitForStart();
        elapsedTime.reset();
        Start();

        while(opModeIsActive() && !isStopRequested()){
            continuousRun(elapsedTime.seconds());
        }
    }

}
