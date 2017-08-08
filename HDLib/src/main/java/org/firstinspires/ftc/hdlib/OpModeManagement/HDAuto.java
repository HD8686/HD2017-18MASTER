package org.firstinspires.ftc.hdlib.OpModeManagement;

/**
 * Created by Akash on 10/20/2016.
 */
public interface HDAuto {

    public void runLoop(double elapsedTime); //elapsedTime is the amount of time since Autonomous has started.

    public void start();

}
