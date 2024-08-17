// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
package frc6328;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Credits: 6328 repository: <a name = "6328 repository" href="https://github.com/Mechanical-Advantage/RobotCode2023/">(Repository Here)</a>
 */
public class DoublePressTracker {
    private double maxLengthSecs = 0.4;

    private final Trigger trigger;
    private final Timer resetTimer = new Timer();
    private DoublePressState state = DoublePressState.IDLE;

    public static Trigger createTrigger(Trigger baseTrigger) {
        var tracker = new DoublePressTracker(baseTrigger);
        return new Trigger(tracker::get);
    }

    public static Trigger createTrigger(Trigger baseTrigger, double maxLengthSecs) {
        var tracker = new DoublePressTracker(baseTrigger);
        tracker.maxLengthSecs = maxLengthSecs;
        return new Trigger(tracker::get);
    }

    private DoublePressTracker(Trigger baseTrigger) {
        trigger = baseTrigger;
    }

    private boolean get() {
        boolean pressed = trigger.getAsBoolean();
        switch (state) {
            case IDLE:
                if (pressed) {
                    state = DoublePressState.FIRST_PRESS;
                    resetTimer.reset();
                    resetTimer.start();
                }
                break;
            case FIRST_PRESS:
                if (!pressed) {
                    if (resetTimer.hasElapsed(maxLengthSecs)) {
                        reset();
                    } else {
                        state = DoublePressState.FIRST_RELEASE;
                    }
                }
                break;
            case FIRST_RELEASE:
                if (pressed) {
                    state = DoublePressState.SECOND_PRESS;
                } else if (resetTimer.hasElapsed(maxLengthSecs)) {
                    reset();
                }
                break;
            case SECOND_PRESS:
                if (!pressed) {
                    reset();
                }
        }
        return state == DoublePressState.SECOND_PRESS;
    }

    private void reset() {
        state = DoublePressState.IDLE;
        resetTimer.stop();
    }

    private enum DoublePressState {
        IDLE,
        FIRST_PRESS,
        FIRST_RELEASE,
        SECOND_PRESS
    }
}
