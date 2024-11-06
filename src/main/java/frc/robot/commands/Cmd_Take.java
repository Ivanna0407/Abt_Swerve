// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Intake_Shooter;

public class Cmd_Take extends Command {
  /** Creates a new Cmd_Take. */
  private final Sub_Intake_Shooter intake_Shooter;
  public Cmd_Take(Sub_Intake_Shooter intake_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake_Shooter=intake_Shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake_Shooter.pieza==false){
      intake_Shooter.SetAgarre(.8);
    }
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
