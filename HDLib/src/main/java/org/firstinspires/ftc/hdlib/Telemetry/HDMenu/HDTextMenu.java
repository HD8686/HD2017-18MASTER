package org.firstinspires.ftc.hdlib.Telemetry.HDMenu;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.Telemetry.HDTelemetry;

import java.util.Map;
import java.util.TreeMap;

/**
 * Created by Akash on 10/16/2016.
 */
public class HDTextMenu extends HDMenuManager {
    private TreeMap<String, Object> choices = new TreeMap<String, Object>();
    private String menuName;
    private HDMenuManager nextMenu;
    private int currSelection = 0;
    private boolean oldLeft = true;
    private boolean oldRight = true;

    public HDTextMenu(String menuName, HDMenuManager nextMenu){
        this.menuName = menuName;
        this.nextMenu = nextMenu;
    }

    @Override
    public void runMenu() {
        while(!HDOpMode.getInstance().gamepad1.a && !HDOpMode.getInstance().isStopRequested()) {
            HDOpMode.getInstance().dashboard.addLibrarySpecificTelemetry(0, "HD Text Menu: %s", menuName);
            HDOpMode.getInstance().dashboard.addLibrarySpecificTelemetry(1, "Press the A button to continue");
            int curLine = 2;
            for(Map.Entry<String, Object> entry: choices.entrySet()){
                if(curLine - 2 == currSelection){
                    HDOpMode.getInstance().dashboard.addLibrarySpecificTelemetry(curLine, "*" + entry.getKey());
                }else{
                    HDOpMode.getInstance().dashboard.addLibrarySpecificTelemetry(curLine, entry.getKey());
                }
                curLine++;
            }
            if(HDOpMode.getInstance().gamepad1.dpad_left != oldLeft && HDOpMode.getInstance().gamepad1.dpad_left){
                currSelection = currSelection - 1;
            }
            if(HDOpMode.getInstance().gamepad1.dpad_right != oldRight && HDOpMode.getInstance().gamepad1.dpad_right){
                currSelection = currSelection + 1;
            }
            if(currSelection < 0) {
                currSelection = 0;
            }
            if(currSelection >= choices.entrySet().size() - 1) {
                currSelection = choices.entrySet().size() - 1;
            }
            oldLeft = HDOpMode.getInstance().gamepad1.dpad_left;
            oldRight = HDOpMode.getInstance().gamepad1.dpad_right;
            HDOpMode.getInstance().idle();
        }

        while(HDOpMode.getInstance().gamepad1.a && !HDOpMode.getInstance().isStopRequested()){
            HDOpMode.getInstance().idle();
            //We wait to make sure they have lifted up the A button until we move on.
        }
    }

    @Override
    public String getSelectionDisplay() {
        return menuName + ": " + choices.keySet().toArray()[currSelection];
    }

    public void addChoice(String choiceName, Object choice){
        choices.put(choiceName, choice);
    }

    public Object getChoice(){
        return choices.values().toArray()[currSelection];
    }

    @Override
    public HDMenuManager getNextMenu() {
        return nextMenu;
    }
}
