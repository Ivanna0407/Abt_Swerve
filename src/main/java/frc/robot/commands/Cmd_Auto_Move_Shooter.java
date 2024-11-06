// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Intake_Shooter;

public class Cmd_Auto_Move_Shooter extends Command {
  /** Creates a new Cmd_StartShooter. */
   private final Sub_Intake_Shooter Shooter;
   double speed;
  public Cmd_Auto_Move_Shooter(Sub_Intake_Shooter shooter,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter=shooter;
    this.speed=speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.Disparar(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Shooter.getShooterspeed()==speed){
      return true;
    }
    else{
      return false;
    }
  }
}
