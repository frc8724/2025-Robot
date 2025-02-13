// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Adapt a magnetic 0-4095 sensor to an offset to 0 and a final range of -2048
 * to +2047.
 */
public class ThirftyAbsMagneticEncoder {
    AnalogInput analogInput;
    int offset;

    public ThirftyAbsMagneticEncoder(int port, int offset) {
        this.analogInput = new AnalogInput(port);
        this.offset = offset;
    }

    public int getValue() {
        return ((this.analogInput.getValue() - this.offset + 2048) % 4096) - 2048;
    }
}
