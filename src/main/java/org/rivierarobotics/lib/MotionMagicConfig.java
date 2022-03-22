package org.rivierarobotics.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * @see MotorUtil#setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)
 * @since 0.3.3
 */
public class MotionMagicConfig extends ControlModeConfig {
    private Integer sCurveStrength = null;

    public ControlModeConfig setSCurveStrength(Integer sCurveStrength) {
        this.sCurveStrength = sCurveStrength;
        return this;
    }

    public Integer getSCurveStrength() {
        return sCurveStrength;
    }

    public ControlModeConfig addStatusFrame(StatusFrameEnhanced statusFrame) {
        return addStatusFrame(statusFrame.value);
    }
}
