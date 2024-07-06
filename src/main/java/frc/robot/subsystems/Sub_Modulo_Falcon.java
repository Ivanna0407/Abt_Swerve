// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class Sub_Modulo_Falcon extends SubsystemBase {
  //Se crean los objetos 
    private  TalonFX drivemotor;
    private  TalonFX turningMotor;



    private   PIDController PIDgiro;

    private  AnalogInput absoluteEncoder;
    private  boolean absoluteEncoderReversed;
    private  double absoluteEncoderOffsetRad;

    //Tiene que llamarse igual 
    //Se crea un constructor, como si fuera un comando o una función para no tener que hacer 4 subsitemas diferentes (1 por modulo)
    public Sub_Modulo_Falcon (int Drive_Motor_ID, int Turn_Motor_ID, boolean Inverted_Drive_Motor, boolean Inverted_Turning_Motor,int Encoder_Absoluto_ID, double offset_encoder_abs,boolean inverted_encoder_abs){
        //Se dan los valores a los objetos que se habían creado antes
        this.absoluteEncoderOffsetRad=offset_encoder_abs;
        this.absoluteEncoderReversed=inverted_encoder_abs;
        absoluteEncoder = new AnalogInput(Encoder_Absoluto_ID);
        turningMotor= new TalonFX(Turn_Motor_ID);
        drivemotor = new TalonFX(Drive_Motor_ID);

        drivemotor.setInverted(Inverted_Drive_Motor);
        turningMotor.setInverted(Inverted_Turning_Motor);

  
        


        PIDgiro= new PIDController(0.02, 0, 0);//Falta checar valores para PID de giro 
        PIDgiro.enableContinuousInput(-Math.PI, Math.PI);//Permite trabajar con los valores de 180 a -180 

        resetEncoders();
        
    }

    public double getDrivePosition(){
        return drivemotor.getPosition().getValueAsDouble()*Swerve.encoder_a_metros;
      }

    public double getTurningPosition(){
        return turningMotor.getPosition().getValueAsDouble()*Swerve.encoder_a_radianes;
      }
    public double getDriveVelocity(){
        return drivemotor.getVelocity().getValueAsDouble()*Swerve.encoder_a_metros_por_segundos;
    }
    public double getTurningVelocity(){
        return turningMotor.getVelocity().getValueAsDouble()*Swerve.encoder_a_radianes_por_segundo;
    }

    public double getAbsoluteEncoderRadians(){
        //Al ser un analog input se tiene que checar que valores muestra 
        double angulo =absoluteEncoder.getVoltage()/ RobotController.getVoltage5V();
        angulo*=2.0*Math.PI;
        angulo-=absoluteEncoderOffsetRad;
        return angulo* (absoluteEncoderReversed ? -1.0:1.0);
    }

    public void resetEncoders(){
        drivemotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        //Se pide un Swerve module state que permite darle una serie de velocidades y posiciónes a los modulos 
        //Si la velocidad del estado es muy poca no se manda nada 
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            drivemotor.set(0);
            turningMotor.set(0);
            return;
        }
        
        state=SwerveModuleState.optimize(state, getState().angle);//330 grados y -30 grados es lo mismo, optimize puede hacer ese calculo 
        //y obtener la ruta más rápida 
        drivemotor.set(state.speedMetersPerSecond/3.5);//3.5 es la velocidad máxima del sistema, se debe checar 
        turningMotor.set(PIDgiro.calculate(getTurningPosition(),state.angle.getRadians()));
    }
    
    public void alto(){
      drivemotor.set(0);
      turningMotor.set(0);
    }
}
