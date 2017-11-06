package org.firstinspires.ftc.hdlib.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDLoopInterface;
import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.RobotHardwareLib.Subsystems.HDDriveHandler;

import java.text.DecimalFormat;


/**
 * Created by Akash on 8/18/2016.
 */
public class HDDashboard implements HDLoopInterface.LoopTimer{
    HDTelemetry curDashboard;
    HardwareMap mHardwareMap;
    private static HDDashboard instance = null;

    private String[] ProgramSpecificDisplay = new String[20];
    private String[] LibrarySpecificDisplay = new String[20];
    private String[] DiagnosticSpecificDisplay = new String[20];
    private int curLine;

    public HDDashboard(HDTelemetry DBInstance){
        instance = this;
        this.curDashboard = DBInstance;
        this.mHardwareMap = HDOpMode.getInstance().hardwareMap;
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.ContinuousRun);
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.InitializeLoop);
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.Start);
    }

    public static HDDashboard getInstance(){
        return instance;
    }

    public void addProgramSpecificTelemetry(int lineNum, String format, Object... args){
        ProgramSpecificDisplay[lineNum] = String.format(format, args);
    }

    public void addLibrarySpecificTelemetry(int lineNum, String format, Object... args){
        LibrarySpecificDisplay[lineNum] = String.format(format, args);
    }

    public void addDiagnosticSpecificTelemetry(int lineNum, String format, Object... args){
        DiagnosticSpecificDisplay[lineNum] = String.format(format, args);
    }

    public void clearProgramSpecificTelemetry(){
        ProgramSpecificDisplay = new String[20];
        curDashboard.clearDisplay();
    }

    public void clearLibrarySpecificTelemetry(){
        LibrarySpecificDisplay = new String[20];
        curDashboard.clearDisplay();
    }

    public void clearDiagnosticSpecificTelemetry(){
        DiagnosticSpecificDisplay = new String[20];
        curDashboard.clearDisplay();
    }

    private void displayCenteredText(String text){
        curDashboard.displayPrintf(curLine, HDTelemetry.textPosition.Centered, text);
        curLine++;
    }

    @Override
    public void InitializeLoopOp() {
        DecimalFormat df = new DecimalFormat("#.##");
        curLine = 0;
        displayCenteredText("HD Library Initialized");
        displayCenteredText("--------------------Library Specific Telemetry--------------------");
        boolean LibrarySpecificDisplayEmpty = true;
        for (int i = 0; i < LibrarySpecificDisplay.length; i++)
        {
            if(LibrarySpecificDisplay[i] != null) {
                LibrarySpecificDisplayEmpty = false;
                displayCenteredText(LibrarySpecificDisplay[i]);
            }
        }
        if(LibrarySpecificDisplayEmpty){
            displayCenteredText("No Library Specific Telemetry");
        }
        displayCenteredText("--------------------Program Specific Telemetry--------------------");
        boolean ProgramSpecificDisplayEmpty = true;
        for (int i = 0; i < ProgramSpecificDisplay.length; i++)
        {
            if(ProgramSpecificDisplay[i] != null) {
                ProgramSpecificDisplayEmpty = false;
                displayCenteredText(ProgramSpecificDisplay[i]);
            }
        }
        if(ProgramSpecificDisplayEmpty){
            displayCenteredText("No Program Specific Telemetry");
        }
        curDashboard.refreshDisplay();
    }

    @Override
    public void StartOp() {
        curDashboard.clearDisplay();
        curDashboard.displayPrintf(0, HDTelemetry.textPosition.Centered, "HD Library Running");
        curDashboard.refreshDisplay();
    }

    @Override
    public void continuousCallOp() {
        DecimalFormat df = new DecimalFormat("#.##");
        curLine = 0;
        displayCenteredText("HD Library Running");
        displayCenteredText("--------------------Library Specific Telemetry--------------------");
        boolean LibrarySpecificDisplayEmpty = true;
        for (int i = 0; i < LibrarySpecificDisplay.length; i++)
        {
            if(LibrarySpecificDisplay[i] != null) {
                LibrarySpecificDisplayEmpty = false;
                displayCenteredText(LibrarySpecificDisplay[i]);
            }
        }
        if(LibrarySpecificDisplayEmpty){
            displayCenteredText("No Library Specific Telemetry");
        }
        displayCenteredText("--------------------Program Specific Telemetry--------------------");
        boolean ProgramSpecificDisplayEmpty = true;
        for (int i = 0; i < ProgramSpecificDisplay.length; i++)
        {
            if(ProgramSpecificDisplay[i] != null) {
                ProgramSpecificDisplayEmpty = false;
                displayCenteredText(ProgramSpecificDisplay[i]);
            }
        }
        if(ProgramSpecificDisplayEmpty){
            displayCenteredText("No Program Specific Telemetry");
        }
        displayCenteredText("--------------------------------Diagnostics--------------------------------");
        displayCenteredText("Program Runtime: " + df.format(HDOpMode.getInstance().elapsedTime.seconds()));
        boolean DiagnosticSpecificDisplayEmpty = true;
        for (int i = 0; i < DiagnosticSpecificDisplay.length; i++)
        {
            if(DiagnosticSpecificDisplay[i] != null) {
                DiagnosticSpecificDisplayEmpty = false;
                displayCenteredText(DiagnosticSpecificDisplay[i]);
            }
        }
        if(DiagnosticSpecificDisplayEmpty){
            displayCenteredText("No Diagnostic Specific Telemetry");
        }
        curDashboard.refreshDisplay();
    }









}
