// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Intake_Shooter extends SubsystemBase {
  /** Creates a new Sub_Intake_Shooter. */
  private final CANSparkMax MotorIntakeD = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax MotorIntakeI = new CANSparkMax(17, MotorType.kBrushless);
  private final TalonFX MotorShooterU = new TalonFX(14);
  private final TalonFX MotorShooterD = new TalonFX(15);

  private final RelativeEncoder EncoderIntakeD = MotorIntakeD.getEncoder();
  private final RelativeEncoder EncoderIntakeI = MotorIntakeI.getEncoder();
  public boolean pieza;
  public Sub_Intake_Shooter() {
   var factorydefaults = new MotorOutputConfigs();
   
    MotorIntakeD.restoreFactoryDefaults();
     MotorIntakeD.setIdleMode(IdleMode.kBrake);
     MotorIntakeD.set(0);

     MotorIntakeI.restoreFactoryDefaults();
     MotorIntakeI.setIdleMode(IdleMode.kBrake);
     MotorIntakeI.set(0);

     MotorShooterU.getConfigurator().apply(factorydefaults);
     MotorShooterU.setNeutralMode(NeutralModeValue.Brake);
     MotorShooterU.setPosition(0);

     MotorShooterD.getConfigurator().apply(factorydefaults);
     MotorShooterD.setNeutralMode(NeutralModeValue.Brake);
     MotorShooterD.setPosition(0);

    EncoderIntakeD.setPosition(0); 
    EncoderIntakeD.setPositionConversionFactor(1/3);

    EncoderIntakeI.setPosition(0); 
    EncoderIntakeI.setPositionConversionFactor(1/3);

    MotorIntakeI.follow(MotorIntakeD);
    MotorIntakeI.setInverted(true);

    MotorShooterU.setInverted(true);
    MotorShooterD.setInverted(true);
  }

    public void SetAgarre (double RightSpeed){
      if (pieza==true && RightSpeed>0){pieza=false;}
      MotorIntakeD.set(RightSpeed);
    }

    public void Disparar (double Disparar){
      MotorShooterU.set(Disparar);
      MotorShooterD.set(Disparar);
    }
    public void Disparar_dos_velocidad (double velocidad_U, double velocidad_D){
      MotorShooterU.set(velocidad_U);
      MotorShooterD.set(velocidad_D);
    }
    public double getShooterspeed(){
      return  MotorShooterD.get();
    }
    public double getintakespeed(){
      return  MotorIntakeD.get();
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Pieza",pieza);
    SmartDashboard.putNumber("Corriente", MotorIntakeD.getOutputCurrent());
    SmartDashboard.putNumber("Corriente_1", MotorIntakeI.getOutputCurrent());
    if (MotorIntakeD.getOutputCurrent()>=52){
      pieza=true;
    }
  }
}
