package org.firstinspires.ftc.hdcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.hdcode.Autonomous.Blue_NonRelic_DoubleGlyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Blue_NonRelic_Glyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Blue_Relic_Glyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Blue_Relic_Multi_Glyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Red_NonRelic_DoubleGlyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Red_NonRelic_Glyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Red_Relic_Glyph_Jewel;
import org.firstinspires.ftc.hdcode.Autonomous.Red_Relic_Multi_Glyph_Jewel;
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
        NONRELICGLYPHJEWEL,
        RELICGLYPHDOUBLEJEWEL,
        RELICGLYPHJEWEL,
        RELICMULTIGLYPHJEWEL,
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
        strategyMenu.addChoice("Non Relic Glyph & Jewel", Strategy.NONRELICGLYPHJEWEL);
        strategyMenu.addChoice("Relic Multi Glyph & Jewel", Strategy.RELICMULTIGLYPHJEWEL);
        strategyMenu.addChoice("Relic Glyph & Jewel", Strategy.RELICGLYPHJEWEL);

        allianceMenu = new HDTextMenu("Alliance", strategyMenu);
        allianceMenu.addChoice("Red Alliance", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue Alliance", Alliance.BLUE_ALLIANCE);

        HDMenuManager.runMenus(allianceMenu);
        delay = delayMenu.getValue();
        alliance = (Alliance) allianceMenu.getChoice();
        strategy = (Strategy) strategyMenu.getChoice();
        Alliance.storeAlliance(hardwareMap.appContext, alliance);

        if(((boolean) autoTransitionToTeleop.getChoice())){
            AutoTransitioner.transitionOnStop(this, "HDTeleop");
        }

        HDMenuManager.displaySelections(allianceMenu, 1);

        switch(strategy){
            case NONRELICGLYPHJEWEL:
                if(alliance == Alliance.RED_ALLIANCE)
                    HDAuto = new Red_NonRelic_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                else
                    HDAuto = new Blue_NonRelic_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                break;
            case RELICMULTIGLYPHJEWEL:
                if(alliance == Alliance.RED_ALLIANCE)
                    HDAuto = new Red_Relic_Multi_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                else
                    HDAuto = new Blue_Relic_Multi_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                break;
            case RELICGLYPHJEWEL:
                if(alliance == Alliance.RED_ALLIANCE)
                    HDAuto = new Red_Relic_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                else
                    HDAuto = new Blue_Relic_Glyph_Jewel(delay, alliance, hardwareMap, dashboard);
                break;
        }

    }

    @Override
    public void initializeLoop() {HDAuto.initializeLoop();}

    @Override
    public void Start() {
        HDAuto.start();
    }

    @Override
    public void continuousRun(double elapsedTime) {
        HDAuto.runLoop(elapsedTime);
    }
}
