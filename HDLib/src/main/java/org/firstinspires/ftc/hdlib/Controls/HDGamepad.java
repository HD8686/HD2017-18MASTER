package org.firstinspires.ftc.hdlib.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.hdlib.OpModeManagement.HDLoopInterface;

/**
 * Created by Akash on 11/1/2016.
 */
public class HDGamepad implements HDLoopInterface.LoopTimer{

    public class gamepadButtons{
        private boolean A;
        private  boolean B;
        private  boolean X;
        private  boolean Y;
        private  boolean DPAD_LEFT;
        private  boolean DPAD_RIGHT;
        private  boolean DPAD_UP;
        private  boolean DPAD_DOWN;
        private  boolean LEFT_BUMPER;
        private boolean RIGHT_BUMPER;
        private boolean RIGHT_TRIGGER;
        private boolean LEFT_TRIGGER;
        private boolean START;

    }

    public enum gamepadButtonChange{
        A,
        B,
        X,
        Y,
        DPAD_LEFT,
        DPAD_RIGHT,
        DPAD_UP,
        DPAD_DOWN,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        RIGHT_TRIGGER,
        LEFT_TRIGGER,
        START
    }

    public interface HDButtonMonitor{
        void buttonChange(HDGamepad instance, gamepadButtonChange button, boolean pressed);
    }

    HDButtonMonitor buttonMonitor;
    Gamepad gamepadInstance;
    gamepadButtons oldGamepad;
    gamepadButtons curGamepad;
    private double triggerPressed = 0.5;

    public HDGamepad(Gamepad gamepad, HDButtonMonitor buttonMonitor){
        this.gamepadInstance = gamepad;
        this.buttonMonitor = buttonMonitor;
        HDLoopInterface.getInstance().register(this, HDLoopInterface.registrationTypes.ContinuousRun);
        oldGamepad = new gamepadButtons();
        curGamepad = new gamepadButtons();
    }

    public void setGamepad(Gamepad gamepad){
        gamepadInstance = gamepad;
    }

    public void setTriggerThreshold(double threshold){
        triggerPressed = threshold;
    }

    private void copyGamepadToClass(Gamepad gamepad, gamepadButtons copy){
        copy.A = gamepad.a;
        copy.B = gamepad.b;
        copy.X = gamepad.x;
        copy.Y = gamepad.y;
        copy.DPAD_DOWN = gamepad.dpad_down;
        copy.DPAD_LEFT = gamepad.dpad_left;
        copy.DPAD_RIGHT = gamepad.dpad_right;
        copy.DPAD_UP = gamepad.dpad_up;
        copy.LEFT_BUMPER = gamepad.left_bumper;
        copy.RIGHT_BUMPER = gamepad.right_bumper;
        copy.RIGHT_TRIGGER = gamepad.right_trigger > triggerPressed;
        copy.LEFT_TRIGGER = gamepad.left_trigger > triggerPressed;
        copy.START = gamepad.start;

    }

    @Override
    public void StartOp() {

    }

    @Override
    public void InitializeLoopOp() {

    }

    @Override
    public void continuousCallOp() {
        copyGamepadToClass(gamepadInstance, curGamepad);
        if(oldGamepad.A != curGamepad.A){
            buttonMonitor.buttonChange(this, gamepadButtonChange.A, curGamepad.A);
        }
        if(oldGamepad.B != curGamepad.B){
            buttonMonitor.buttonChange(this, gamepadButtonChange.B, curGamepad.B);
        }
        if(oldGamepad.X != curGamepad.X){
            buttonMonitor.buttonChange(this, gamepadButtonChange.X, curGamepad.X);
        }
        if(oldGamepad.Y != curGamepad.Y){
            buttonMonitor.buttonChange(this, gamepadButtonChange.Y, curGamepad.Y);
        }
        if(oldGamepad.DPAD_UP != curGamepad.DPAD_UP){
            buttonMonitor.buttonChange(this, gamepadButtonChange.DPAD_UP, curGamepad.DPAD_UP);
        }
        if(oldGamepad.DPAD_DOWN != curGamepad.DPAD_DOWN){
            buttonMonitor.buttonChange(this, gamepadButtonChange.DPAD_DOWN, curGamepad.DPAD_DOWN);
        }
        if(oldGamepad.DPAD_LEFT != curGamepad.DPAD_LEFT){
            buttonMonitor.buttonChange(this, gamepadButtonChange.DPAD_LEFT, curGamepad.DPAD_LEFT);
        }
        if(oldGamepad.DPAD_RIGHT != curGamepad.DPAD_RIGHT){
            buttonMonitor.buttonChange(this, gamepadButtonChange.DPAD_RIGHT, curGamepad.DPAD_RIGHT);
        }
        if(oldGamepad.RIGHT_BUMPER != curGamepad.RIGHT_BUMPER){
            buttonMonitor.buttonChange(this, gamepadButtonChange.RIGHT_BUMPER, curGamepad.RIGHT_BUMPER);
        }
        if(oldGamepad.LEFT_BUMPER != curGamepad.LEFT_BUMPER){
            buttonMonitor.buttonChange(this, gamepadButtonChange.LEFT_BUMPER, curGamepad.LEFT_BUMPER);
        }
        if(oldGamepad.LEFT_TRIGGER != curGamepad.LEFT_TRIGGER){
            buttonMonitor.buttonChange(this, gamepadButtonChange.LEFT_TRIGGER, curGamepad.LEFT_TRIGGER);
        }
        if(oldGamepad.RIGHT_TRIGGER != curGamepad.RIGHT_TRIGGER){
            buttonMonitor.buttonChange(this, gamepadButtonChange.RIGHT_TRIGGER, curGamepad.RIGHT_TRIGGER);
        }
        if(oldGamepad.START != curGamepad.START){
            buttonMonitor.buttonChange(this, gamepadButtonChange.START, curGamepad.START);
        }
        copyGamepadToClass(gamepadInstance, oldGamepad);
    }

}
