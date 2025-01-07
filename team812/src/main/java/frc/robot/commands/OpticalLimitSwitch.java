// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
/** Add your docs here. */

public class OpticalLimitSwitch {
    private DigitalInput limitSwitch;

    public OpticalLimitSwitch(int channel) {
        limitSwitch = new DigitalInput(channel);
    }

    public boolean isClosed() {
        return limitSwitch.get();
    }

    public boolean isOpen() {
        return !limitSwitch.get();
    }
}
