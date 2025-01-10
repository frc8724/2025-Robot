// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public interface IMayhemCANSparkMax {
    CANSparkMax getMotor();

    void setInverted(boolean b);

    void follow(MayhemCANSparkMax m);

    void setVBusPower(double d);
}
