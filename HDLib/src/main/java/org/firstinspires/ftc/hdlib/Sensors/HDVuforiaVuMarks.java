package org.firstinspires.ftc.hdlib.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by FIRSTMentor on 11/17/2017.
 */

public class HDVuforiaVuMarks {

    OpenGLMatrix lastLocation = null;
    RelicRecoveryVuMark vuMark;
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    public HDVuforiaVuMarks(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZRKRX7/////AAAAGcjcJ3tvlkTjn8e4xEVqyXhl9BWco/Vc+Xkv2384x9sMZzG3BzUJLKDyqcaA0txYsQo00NqXurmoRHm90/OJcQYIkWWV9plQZ6nLVv07yFl8PqTGnRNVazOgi1IzxPWqGBznN5sGboRXvAUn+VQsdyN3e0KU6lB/Cl5vre2Wi7DtntufCGNcdqU0pdN9LlKpQr6byV4zYQ7p81g3cEHY5AkI3egvEy+thpk3NjyUMeFK9SeCfIKgDvHEx9G4bCBFmmj/+knydr5BM0bJ0Jh5GmNRLNErQewCvA+SET/K2jYRZvEcpvZTxSPQS4ho+JBxC4pk7i4KmPwrWHHnHSCqmwboLv301r2njWgHcs6UZK1+";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public void activateTracking(){
        relicTrackables.activate();
    }

    public void deactivateTracking(){
        relicTrackables.deactivate();
    }

    public RelicRecoveryVuMark scan(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public VectorF getTranslation(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                return trans;
            }else{
                return null;
            }
        }else{
            return null;
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void flash(boolean on){
        CameraDevice.getInstance().setFlashTorchMode(on);
    }

}
