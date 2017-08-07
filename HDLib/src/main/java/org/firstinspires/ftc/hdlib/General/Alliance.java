package org.firstinspires.ftc.hdlib.General;

import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * Created by Akash on 10/20/2016.
 */
public enum Alliance {
    RED_ALLIANCE,
    BLUE_ALLIANCE,
    UNKNOWN;

    public static Alliance stringToAlliance (String myEnumString) {
        try {
            return valueOf(myEnumString);
        } catch (Exception ex) {
            return UNKNOWN;
        }
    }

    public static void storeAlliance(Context context, Alliance curAlliance){
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("HDAlliance.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(curAlliance.toString());
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    public static Alliance retrieveAlliance(Context context){
        String allianceString = "";
        try {
            InputStream inputStream = context.openFileInput("HDAlliance.txt");
            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ( (receiveString = bufferedReader.readLine()) != null ) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                allianceString = stringBuilder.toString();
            }
        }
        catch (FileNotFoundException e) {
            Log.e("AllianceReader", "HDAlliance File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("AllianceReader", "Can not read HDAlliance file: " + e.toString());
        }
        if(Alliance.stringToAlliance(allianceString) == Alliance.UNKNOWN) {
            throw new NullPointerException("Can't retrieve Alliance from Autonomous.");
        }

        return Alliance.stringToAlliance(allianceString);
    }
}
