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
import edu.wpi.first.networktables.StructArrayPublisher;



public class Sub_Swerve extends SubsystemBase {
  //En este subsistema se unen los 4 modulos y el giroscopio 
  private final Sub_Modulo Modulo_1 = new Sub_Modulo(1, 22, true, false, 9,  0, false);
  private final Sub_Modulo Modulo_2 = new Sub_Modulo(3, 4, false, false, 10, 0, false);
  private final Sub_Modulo Modulo_3 = new Sub_Modulo(5, 6, false, false, 11, 0, false);
  private final Sub_Modulo Modulo_4 = new Sub_Modulo(7, 8, true, false, 12,0 , false);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final StructArrayPublisher<SwerveModuleState> publisher;
  

  

  public Sub_Swerve() {
    new Thread(()->{try {Thread.sleep(1000); zeroHeading();}catch(Exception e ){}}).start();
    //publisher = new NetworkTableInstance.getDefault().getStructArrayTpoic("/Swervemodulestates", SwerveModuleState.struct).publish();
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Heading", getHeadding());
    SmartDashboard.putNumber("Abs_1", Modulo_1.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_2", Modulo_2.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_3", Modulo_3.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Abs_4", Modulo_4.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Roll", gyro.getRoll());
    SmartDashboard.putNumber("Tracción_2", Modulo_2.getDriveVelocity());
    SmartDashboard.putNumber("Giro_2", Modulo_2.getTurningVelocity());
    SmartDashboard.putNumber("Tracción_1", Modulo_1.getDriveVelocity());
    SmartDashboard.putNumber("Giro_1", Modulo_1.getTurningVelocity());
    publisher.set(new SwerveModuleState[]{Modulo_1.getState(),Modulo_2.getState(),Modulo_3.getState(),Modulo_4.getState()});
    
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, 2.5);
    Modulo_1.setDesiredState(desiredModuleStates[0]);
    Modulo_2.setDesiredState(desiredModuleStates[1]);
    Modulo_3.setDesiredState(desiredModuleStates[2]);
    Modulo_4.setDesiredState(desiredModuleStates[3]);
  }

  public void resetAllEncoders(){
    Modulo_1.resetEncoders();
    Modulo_2.resetEncoders();
    Modulo_3.resetEncoders();
    Modulo_4.resetEncoders();
  }

  public void setSpecificState(SwerveModuleState specificState,SwerveModuleState specificState_1,SwerveModuleState specificState_2,SwerveModuleState specificState_3){
    Modulo_1.setDesiredState(specificState);
    Modulo_2.setDesiredState(specificState_1);
    Modulo_3.setDesiredState(specificState_2);
    Modulo_4.setDesiredState(specificState_3);
  }
}
