// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Swerve;

public class Cmd_vision extends Command {
  /** Creates a new Cmd_vision. */
  private final Sub_Swerve  sub_Swerve;
  public Cmd_vision(Sub_Swerve sub_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub_Swerve=sub_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
