// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Sub_Swerve extends SubsystemBase {
  //En este subsistema se unen los 4 modulos y el giroscopio 
  private final Sub_Modulo Modulo_1 = new Sub_Modulo(1, 2, false, false, 9, .4321, false);
  private final Sub_Modulo Modulo_2 = new Sub_Modulo(3, 4, false, false, 10, -2.89, false);
  private final Sub_Modulo Modulo_3 = new Sub_Modulo(5, 6, false, false, 11, 0.4596, false);
  private final Sub_Modulo Modulo_4 = new Sub_Modulo(7, 8, false, false, 12, .1784, false);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  

  public Sub_Swerve() {
    new Thread(()->{try {Thread.sleep(1000); zeroHeading();}catch(Exception e ){}}).start();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Heading", getHeadding());
    SmartDashboard.putNumber("Encoderabs", Modulo_1.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Velocidad turning", Modulo_1.getTurningVelocity());
    SmartDashboard.putNumber("Abs_1", Modulo_1.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_2", Modulo_2.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_3", Modulo_3.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_4", Modulo_4.getAbsoluteEncoderRadians());
    
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeadding(){
    return Math.IEEEremainder(gyro.getAngle(),360);
  }

  public Rotation2d get2Drotation(){
    //Permite cambiar de angulos a un objeto de Rotation 2D
    return Rotation2d.fromDegrees(getHeadding());

  }

  public void stopModules(){
    Modulo_1.alto();
    Modulo_2.alto();
    Modulo_3.alto();
    Modulo_4.alto();
  }

  public void setModuleStates(SwerveModuleState[] desiredModuleStates){
    //Se genera un arreglo de swerve module state para poder mandarlos a los diferentes modulos de acuerdo a posición 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, 1.5);
    Modulo_1.setDesiredState(desiredModuleStates[0]);
    Modulo_2.setDesiredState(desiredModuleStates[1]);
    Modulo_3.setDesiredState(desiredModuleStates[2]);
    Modulo_4.setDesiredState(desiredModuleStates[3]);
  }
}
