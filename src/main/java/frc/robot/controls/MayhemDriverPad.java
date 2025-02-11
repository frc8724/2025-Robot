package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.JoystickAxisButton.Direction;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class MayhemDriverPad {

        public static final int GAMEPAD_F310_A_BUTTON = 1;
        public static final int GAMEPAD_F310_B_BUTTON = 2;
        public static final int GAMEPAD_F310_X_BUTTON = 3;
        public static final int GAMEPAD_F310_Y_BUTTON = 4;
        public static final int GAMEPAD_F310_LEFT_BUTTON = 5;
        public static final int GAMEPAD_F310_RIGHT_BUTTON = 6;
        public static final int GAMEPAD_F310_BACK_BUTTON = 7;
        public static final int GAMEPAD_F310_START_BUTTON = 8;
        public static final int GAMEPAD_F310_LEFT_STICK_BUTTON = 9;
        public static final int GAMEPAD_F310_RIGHT_STICK_BUTTON = 10;

        public static final int GAMEPAD_F310_LEFT_X_AXIS = 0;
        public static final int GAMEPAD_F310_LEFT_Y_AXIS = 1;
        public static final int GAMEPAD_F310_LEFT_TRIGGER = 2;
        public static final int GAMEPAD_F310_RIGHT_TRIGGER = 3;
        public static final int GAMEPAD_F310_RIGHT_X_AXIS = 4;
        public static final int GAMEPAD_F310_RIGHT_Y_AXIS = 5;

        public final Joystick m_joystick;

        public MayhemDriverPad(int port) {
                m_joystick = new Joystick(port);
        }

        public Trigger Button(int button) {
                return new JoystickButton(m_joystick, button);
        }

        public Trigger PovButton(int button) {
                return new JoystickPOVButton(m_joystick, button);
        }

        public Trigger AxisButton(int axis, Direction dir) {
                return new JoystickAxisButton(m_joystick, axis, dir);
        }

        public Supplier<Double> AxisSupplier(int axis) {
                return () -> m_joystick.getRawAxis(axis);
        }
}