// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Swerve;

public class Cmd_Auto_Move extends Command {
  /** Creates a new Cmd_Auto_Move. */
  private final Sub_Swerve Sub_swerve;
  private final double second;
  boolean timer= false;
  
  public Cmd_Auto_Move(Sub_Swerve Sub_Swerve, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Sub_swerve=Sub_Swerve;
    this.second=seconds;
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Sub_swerve.setSpecificState(new SwerveModuleState(.5,Rotation2d.fromRadians(0)), new SwerveModuleState(.5,Rotation2d.fromRadians(0)), new SwerveModuleState(.5,Rotation2d.fromRadians(0)), new SwerveModuleState(.5,Rotation2d.fromRadians(0)));
    Timer.delay(second);
    Sub_swerve.setSpecificState(new SwerveModuleState(0,Rotation2d.fromRadians(0)), new SwerveModuleState(0,Rotation2d.fromRadians(0)), new SwerveModuleState(0,Rotation2d.fromRadians(0)), new SwerveModuleState(0,Rotation2d.fromRadians(0)));
    timer=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer==true){return true;}
    else{
    return false;
    }
  }
}
