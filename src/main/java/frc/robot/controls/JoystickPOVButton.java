/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * @author Team1519
 */
public class JoystickPOVButton extends Trigger {
    public static final int NORTH = 0;
    public static final int NORTHEAST = 45;
    public static final int EAST = 90;
    public static final int SOUTHEAST = 135;
    public static final int SOUTH = 180;
    public static final int SOUTHWEST = 225;
    public static final int WEST = 270;
    public static final int NORTHWEST = 315;

    private Joystick joystick;
    private int desiredPOV;

    public JoystickPOVButton(Joystick stick, int newDesiredPOV) {
        super(() -> stick.getPOV() == newDesiredPOV);
        joystick = stick;
        desiredPOV = newDesiredPOV;
    }

    // @Override
    public boolean get() {
        return (joystick.getPOV() == desiredPOV);
    }
}
