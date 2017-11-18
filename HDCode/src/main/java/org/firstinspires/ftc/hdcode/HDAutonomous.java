package org.firstinspires.ftc.hdcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.hdcode.Autonomous.Auto1;
import org.firstinspires.ftc.hdcode.Autonomous.Auto2;
import org.firstinspires.ftc.hdlib.General.Alliance;
import org.firstinspires.ftc.hdlib.OpModeManagement.AutoTransitioner;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDAuto;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.Telemetry.HDMenu.HDMenuManager;
import org.firstinspires.ftc.hdlib.Telemetry.HDMenu.HDNumberMenu;
import org.firstinspires.ftc.hdlib.Telemetry.HDMenu.HDTextMenu;

/**
 * Created by Akash on 9/23/2017.
 */

@Autonomous
public class HDAutonomous extends HDOpMode{

    private enum Strategy {
        AUTO1,
        AUTO2
    }

    private HDAuto HDAuto;
    private double delay = 0.0;
    private Strategy strategy;
    private Alliance alliance = Alliance.RED_ALLIANCE;

    @Override
    public void initialize() {
        HDNumberMenu delayMenu;
        HDTextMenu strategyMenu;
        HDTextMenu allianceMenu;
        HDTextMenu autoTransitionToTeleop;

        autoTransitionToTeleop = new HDTextMenu("Auto Teleop Transition?", null);
        autoTransitionToTeleop.addChoice("Yes!", true);
        autoTransitionToTeleop.addChoice("No", false);

        delayMenu = new HDNumberMenu("Delay", 0, 30, 1, 0, "Seconds", autoTransitionToTeleop);

        strategyMenu = new HDTextMenu("Strategy", delayMenu);
        strategyMenu.addChoice("Auto 1", Strategy.AUTO1);
        strategyMenu.addChoice("Auto 2", Strategy.AUTO2);

        allianceMenu = new HDTextMenu("Alliance", strategyMenu);
        allianceMenu.addChoice("Red Alliance", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue Alliance", Alliance.BLUE_ALLIANCE);

        HDMenuManager.runMenus(allianceMenu);
        delay = delayMenu.getValue();
        alliance = (Alliance) allianceMenu.getChoice();
        strategy = (Strategy) strategyMenu.getChoice();
        Alliance.storeAlliance(hardwareMap.appContext, alliance);

        if(((boolean) autoTransitionToTeleop.getChoice()) == true){
            AutoTransitioner.transitionOnStop(this, "HDTeleop");
        }

        HDMenuManager.displaySelections(allianceMenu, 1);

        switch(strategy){
            case AUTO1:
                HDAuto = new Auto1(delay, alliance, hardwareMap);
                break;
            case AUTO2:
                HDAuto = new Auto2(delay,alliance, hardwareMap);
                break;
        }

    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void Start() {
        HDAuto.start();
    }

    @Override
    public void continuousRun(double elapsedTime) {
        HDAuto.runLoop(elapsedTime);
    }
}
