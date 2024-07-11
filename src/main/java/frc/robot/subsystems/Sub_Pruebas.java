// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Pruebas extends SubsystemBase {
  /** Creates a new Sub_Pruebas. */
  private final CANSparkMax Motor_1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax Motor_2 = new CANSparkMax(1, MotorType.kBrushless);

  public Sub_Pruebas() {
    Motor_1.restoreFactoryDefaults();
    Motor_2.restoreFactoryDefaults();

    Motor_1.setIdleMode(IdleMode.kBrake);
    Motor_2.setIdleMode(IdleMode.kBrake);

    Motor_1.set(0);
    Motor_2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor_1", Motor_1.get());
    SmartDashboard.putNumber("Motor_2", Motor_2.get());
  }


  public void setSpeed_Motor_1(double speed){
    Motor_1.set(speed*.3);
  }

  public void setSpeed_Motor_2(double speed){
    Motor_2.set(speed*.3);
  }
}
