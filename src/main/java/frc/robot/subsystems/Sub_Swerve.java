// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;





public class Sub_Swerve extends SubsystemBase {
  //En este subsistema se unen los 4 modulos y el giroscopio 
  private final Sub_Modulo Modulo_1 = new Sub_Modulo(7, 8, false, true, 12,  0, true);
  private final Sub_Modulo Modulo_2 = new Sub_Modulo(5, 6, false, true, 11, 0, true);
  private final Sub_Modulo Modulo_3 = new Sub_Modulo(1, 22, false, true, 9, 0, true);
  private final Sub_Modulo Modulo_4 = new Sub_Modulo(3, 4, false , true, 10,0 , true);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final StructArrayPublisher<SwerveModuleState> publisher;
  private final SwerveDriveOdometry odometry= new SwerveDriveOdometry(Swerve.swervekinematics,gyro.getRotation2d(), getModulePositions());
  private Field2d field= new Field2d();
  

  

  public Sub_Swerve() {
    new Thread(()->{try {Thread.sleep(1000); zeroHeading();}catch(Exception e ){}}).start();
    CameraServer.startAutomaticCapture("Sprite_Cam",0);
    
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
    publisher.set(new SwerveModuleState[]{Modulo_1.getState(),Modulo_2.getState(),Modulo_3.getState(),Modulo_4.getState()}); 
    field.setRobotPose(getPose());   
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

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  
  public void stopModules(){
    Modulo_1.alto();
    Modulo_2.alto();
    Modulo_3.alto();
    Modulo_4.alto();
  }

  public void setModuleStates(SwerveModuleState[] desiredModuleStates){
    //Se genera un arreglo de swerve module state para poder mandarlos a los diferentes modulos de acuerdo a posición 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, 3);
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

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0]=Modulo_1.getModulePosition();
    positions[1]=Modulo_2.getModulePosition();
    positions[2]=Modulo_3.getModulePosition();
    positions[3]=Modulo_4.getModulePosition();
    return positions;
    }
  }

