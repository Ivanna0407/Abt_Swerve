// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Sub_Modulo extends SubsystemBase {
  /** Creates a new Sub_Modulo. */
    private final CANSparkMax driveMotor;
    private final   CANSparkMax turningMotor;

    private final  RelativeEncoder driveEncoder;
    private final  RelativeEncoder turningEncoder;

    private final  PIDController PIDgiro;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    //Tiene que llamarse igual 
    public Sub_Modulo (int Drive_Motor_ID, int Turn_Motor_ID, boolean Inverted_Drive_Motor, boolean Inverted_Turning_Motor,int Encoder_Absoluto_ID, double offset_encoder_abs,boolean inverted_encoder_abs){
        this.absoluteEncoderOffsetRad=offset_encoder_abs;
        this.absoluteEncoderReversed=inverted_encoder_abs;
        absoluteEncoder = new AnalogInput(Encoder_Absoluto_ID);
        turningMotor= new CANSparkMax(Turn_Motor_ID, MotorType.kBrushless);
        driveMotor= new CANSparkMax(Drive_Motor_ID, MotorType.kBrushless);

        driveMotor.setInverted(Inverted_Drive_Motor);
        turningMotor.setInverted(Inverted_Turning_Motor);

        driveEncoder=driveMotor.getEncoder();
        turningEncoder=turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Swerve.drive_motor_gear_ratio);
        driveEncoder.setVelocityConversionFactor(Swerve.encoder_a_metros_por_segundos);

        turningEncoder.setPositionConversionFactor(Swerve.encoder_a_radianes);
        turningEncoder.setVelocityConversionFactor(Swerve.encoder_a_radianes_por_segundo);

        PIDgiro= new PIDController(0.02, 0, 0);//Falta checar valores para PID de giro 
        PIDgiro.enableContinuousInput(-Math.PI, Math.PI);

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
        double angulo =absoluteEncoder.getVoltage()/ RobotController.getVoltage5V();
        angulo*=2.0*Math.PI;
        angulo-=absoluteEncoderOffsetRad;
        return angulo* (absoluteEncoderReversed ? -1.0:1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        /* 
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        }
        */
        state=SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond/3.5);//3.5 es la velocidad máxima del sistema, se debe checar 
        turningMotor.set(PIDgiro.calculate(getTurningPosition(),state.angle.getRadians()));
    }
    
    public void alto(){
      driveMotor.set(0);
      turningMotor.set(0);
    }
}


