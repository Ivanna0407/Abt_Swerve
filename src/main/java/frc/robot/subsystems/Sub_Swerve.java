// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class Sub_Swerve extends SubsystemBase {
  private final Sub_Modulo Modulo_1 = new Sub_Modulo(1, 2, false, false, 0, 0, false);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public Sub_Swerve() {
    new Thread(()->{try {Thread.sleep(1000); zeroHeading();}catch(Exception e ){}}).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Heading", getHeadding());
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeadding(){
    return Math.IEEEremainder(gyro.getAngle(),360);
  }

  public Rotation2d get2Drotation(){
    return Rotation2d.fromDegrees(getHeadding());

  }

  public void stopModules(){
    Modulo_1.alto();
  }

  public void setModuleStates(SwerveModuleState[] desiredModuleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, 3.5);
    Modulo_1.setDesiredState(desiredModuleStates[0]);
  }
}
