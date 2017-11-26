
package org.firstinspires.ftc.hdlib.Telemetry;


import android.app.Activity;
import android.text.TextPaint;
import android.widget.TextView;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDOpMode;
import org.firstinspires.ftc.hdlib.R;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class HDTelemetry
{
    TextPaint mPaint;
    public static final int SCREEN_WIDTH = 667;
    public static final int MAX_NUM_TEXTLINES = 30;
    private static final String displayKeyFormat = "%02d";
    private static Telemetry telemetry = null;
    private static String[] display = new String[MAX_NUM_TEXTLINES];
    public enum textPosition{
        Left,
        Right,
        Centered
    }


    public HDTelemetry(Telemetry telemetry)
    {
        mPaint = ((TextView) ((Activity) HDOpMode.getInstance().hardwareMap.appContext).findViewById(R.id.textOpMode)).getPaint();
        this.telemetry = telemetry;
        telemetry.clearAll();
        clearDisplay();
    }


    public void displayPrintf(int lineNum, textPosition tP, String format, Object... args)
    {
        display[lineNum] = String.format(format, args);
        if (lineNum >= 0 && lineNum < display.length)
        {
            if(tP == textPosition.Centered){
                display[lineNum] = centerText(mPaint, SCREEN_WIDTH, display[lineNum]);
                telemetry.addData(String.format(displayKeyFormat, lineNum), display[lineNum]);
            } else if(tP == textPosition.Right){
                display[lineNum] = justifyTextRight(mPaint, SCREEN_WIDTH, display[lineNum]);
                telemetry.addData(String.format(displayKeyFormat, lineNum), display[lineNum]);
            } else if(tP == textPosition.Left){
                telemetry.addData(String.format(displayKeyFormat, lineNum), display[lineNum]);
            }

        }
    }





    public void clearDisplay()
    {
        for (int i = 0; i < display.length; i++)
        {
            display[i] = "";
        }
        refreshDisplay();
    }


    public void refreshDisplay()
    {
        telemetry.clearAll();
        for (int i = 0; i < display.length; i++)
        {
            telemetry.addData(String.format(displayKeyFormat, i), display[i]);
        }
        telemetry.update();
    }

    private String centerText(TextPaint paint, float width, String text)
    {
        float textWidth = paint.measureText(text);
        int paddingSpaces = Math.round((width - textWidth)/2/paint.measureText(" "));
        String format = "%" + (paddingSpaces + text.length()) + "s";
        return String.format(format, text);
    }

    private String justifyTextRight(TextPaint paint, float width, String text)
    {
        float textWidth = paint.measureText(text);
        int paddingSpaces = Math.round((width - textWidth)/paint.measureText(" "));
        String format = "%" + (paddingSpaces + text.length()) + "s";
        return String.format(format, text);
    }

}