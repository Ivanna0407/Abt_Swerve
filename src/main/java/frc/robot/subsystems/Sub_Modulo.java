// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class Sub_Modulo extends SubsystemBase {
  //Se crean los objetos 
    private final CANSparkMax driveMotor;
    private final   CANSparkMax turningMotor;
    private final  RelativeEncoder driveEncoder;
    private final  RelativeEncoder turningEncoder;
    private final  PIDController PIDgiro;
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    //Tiene que llamarse igual 
    //Se crea un constructor, como si fuera un comando o una función para no tener que hacer 4 subsitemas diferentes (1 por modulo)
    public Sub_Modulo (int Drive_Motor_ID, int Turn_Motor_ID, boolean Inverted_Drive_Motor, boolean Inverted_Turning_Motor,int Encoder_Absoluto_ID, double offset_encoder_abs,boolean inverted_encoder_abs){
        //Se dan los valores a los objetos que se habían creado antes
        this.absoluteEncoderOffsetRad=offset_encoder_abs;
        this.absoluteEncoderReversed=inverted_encoder_abs;
        absoluteEncoder = new CANcoder(Encoder_Absoluto_ID);
        turningMotor= new CANSparkMax(Turn_Motor_ID, MotorType.kBrushless);
        driveMotor= new CANSparkMax(Drive_Motor_ID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(Inverted_Drive_Motor);
        turningMotor.setInverted(Inverted_Turning_Motor);

        driveEncoder=driveMotor.getEncoder();
        turningEncoder=turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Swerve.drive_motor_gear_ratio); //Gear ratio tomado de SDS se encuentra en la parte de constants
        driveEncoder.setVelocityConversionFactor(Swerve.encoder_a_metros_por_segundos);//Se da por el gearratio y la llanta dividendolo por 6'

        turningEncoder.setPositionConversionFactor(Swerve.encoder_a_radianes);//Radianes son más exactos que los angulos 
        turningEncoder.setVelocityConversionFactor(Swerve.encoder_a_radianes_por_segundo);

        PIDgiro= new PIDController(.1, 0, 0);//Falta checar valores para PID de giro 
        PIDgiro.enableContinuousInput(-Math.PI, Math.PI);//Permite trabajar con los valores de 180 a -180 

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        resetEncoders();
        
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
      }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
      }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadians(){
        //Al ser un analog input se tiene que checar que valores muestra 
        double angulo =(absoluteEncoder.getAbsolutePosition().getValueAsDouble()*2* Math.PI);
        angulo-=absoluteEncoderOffsetRad; 
        if (angulo > 2 * Math.PI){
          angulo -= 2* Math.PI;
        }
    
        if (angulo < 0){
          angulo += 2 * Math.PI;
        }
        
        if (absoluteEncoderReversed){
          angulo = 2 * Math.PI - angulo;
        }

        return angulo;
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        //Se pide un Swerve module state que permite darle una serie de velocidades y posiciónes a los modulos 
        //Si la velocidad del estado es muy poca no se manda nada 
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        }
        
        state=SwerveModuleState.optimize(state, getState().angle);//330 grados y -30 grados es lo mismo, optimize puede hacer ese calculo 
        //y obtener la ruta más rápida 
        driveMotor.set(state.speedMetersPerSecond/3);//3.5 es la velocidad máxima del sistema, se debe checar 
        turningMotor.set(PIDgiro.calculate(getTurningPosition(),state.angle.getRadians()));
    }
    
    public void alto(){
      driveMotor.set(0);
      turningMotor.set(0);
    }
}


