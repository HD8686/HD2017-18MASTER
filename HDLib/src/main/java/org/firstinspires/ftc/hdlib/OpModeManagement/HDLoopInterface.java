package org.firstinspires.ftc.hdlib.OpModeManagement;

import java.util.HashSet;
import java.util.Set;

/**
 * Created by akash on 5/9/2016.
 */
public class HDLoopInterface {

    private static HDLoopInterface instance = null;
    Set<LoopTimer> InitializeLoopHS = new HashSet<LoopTimer>();
    Set<LoopTimer> StartHS = new HashSet<LoopTimer>();
    Set<LoopTimer> ContinuousRunHS = new HashSet<LoopTimer>();

    public HDLoopInterface(){
        instance = this;
    }


    public static HDLoopInterface getInstance()
    {
        return instance;
    }   //getInstance


    public interface LoopTimer
    {
        void continuousCallOp();

        void StartOp();

        void InitializeLoopOp();
    }

    public void register(LoopTimer lT, registrationTypes rT){
        switch (rT){
            case InitializeLoop:
                InitializeLoopHS.add(lT);
                break;
            case Start:
                StartHS.add(lT);
                break;
            case ContinuousRun:
                ContinuousRunHS.add(lT);
                break;
        }
    }

    public void deregister(LoopTimer lT, registrationTypes rT){
        switch (rT){
            case InitializeLoop:
                InitializeLoopHS.remove(lT);
                break;
            case Start:
                StartHS.remove(lT);
                break;
            case ContinuousRun:
                ContinuousRunHS.remove(lT);
                break;
        }
    }


    public void runContinuousRunInterface(){
        for(LoopTimer tempLoop: ContinuousRunHS){
            tempLoop.continuousCallOp();
        }
    }
    public void runInitializeLoopInterface(){
        for(LoopTimer tempLoop: InitializeLoopHS){
            tempLoop.InitializeLoopOp();
        }
    }
    public void runStartInterface(){
        for(LoopTimer tempLoop: StartHS){
            tempLoop.StartOp();
        }
    }

    public enum registrationTypes{
        InitializeLoop, Start, ContinuousRun,
    }

}


