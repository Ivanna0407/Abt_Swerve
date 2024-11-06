// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Intake_Shooter;

public class Cmd_Intake_shoot extends Command {
  /** Creates a new Cmd_Intake_shoot. */
  private final Sub_Intake_Shooter Sub_Intake_Shooter;
  private final Supplier<Boolean> X, Y,B,bumper;
  private final Supplier<Double> Trigger_S;
  public Cmd_Intake_shoot(Sub_Intake_Shooter Sub_Intake_Shooter, Supplier<Boolean> X, Supplier<Boolean> Y, Supplier<Double> Trigger_S,Supplier<Boolean> B,Supplier<Boolean>Bumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Sub_Intake_Shooter= Sub_Intake_Shooter;
    this.X=X;
    this.Y=Y;
    this.Trigger_S=Trigger_S;
    this.B=B;
    this.bumper=Bumper;
    addRequirements(Sub_Intake_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Trigger_S.get();
    if (bumper.get()){speed=speed*-.6;}
    Sub_Intake_Shooter.Disparar(-speed);
    
    
    //Sub_Intake_Shooter.Disparar(reversa);
    if (X.get()){
      Sub_Intake_Shooter.SetAgarre(-.8);
    }
    else{if (Y.get()){
      Sub_Intake_Shooter.SetAgarre(.8);
    }
    else{Sub_Intake_Shooter.SetAgarre(0);}}
    if(B.get()){
      Sub_Intake_Shooter.Disparar_dos_velocidad(-.6, -.3);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Sub_Intake_Shooter.SetAgarre(0);
    Sub_Intake_Shooter.Disparar(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
