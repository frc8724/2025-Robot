// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class MayhemCANSparkMax implements IMayhemCANSparkMax {
    CANSparkMax motor;

    public MayhemCANSparkMax(int id, MotorType type) {
        motor = new CANSparkMax(id, type);
        motor.restoreFactoryDefaults();
    }

    @Override
    public void setInverted(boolean b) {
        motor.setInverted(b);
    }

    @Override
    public void follow(MayhemCANSparkMax leader) {
        motor.follow(leader.motor);
    }

    @Override
    public CANSparkMax getMotor() {
        return motor;
    }

    @Override
    public void setVBusPower(double d) {
        motor.set(d);
    }
}
