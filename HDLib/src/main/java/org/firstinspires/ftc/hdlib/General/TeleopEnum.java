package org.firstinspires.ftc.hdlib.General;

/**
 * Created by FIRSTMentor on 4/16/2018.
 */

public class TeleopEnum {

    public enum boxPosition{
        STOWED,
        FLAT,
        OUT,
    }

    public enum liftHeight{
        GROUND,
        HIGH,
    }

    public enum driveMode{
        FIELD_CENTRIC_DRIVE,
        HALO_DRIVE,
        TANK_DRIVE;

        public driveMode getNext() {
            return this.ordinal() < driveMode.values().length - 1
                    ? driveMode.values()[this.ordinal() + 1]
                    : driveMode.values()[0];
        }
    }


}
