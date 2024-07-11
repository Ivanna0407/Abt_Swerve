// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Pruebas;

public class Cmd_Pruebas extends Command {
  /** Creates a new Cmd_Pruebas. */
  private final Sub_Pruebas Sub_Pruebas;
  private final Supplier<Double> RT, LT;
  public Cmd_Pruebas(Sub_Pruebas Sub_Pruebas, Supplier<Double> RT,Supplier<Double>LT) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Sub_Pruebas=Sub_Pruebas;
    this.RT=RT;
    this.LT=LT;
    addRequirements(Sub_Pruebas);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Sub_Pruebas.setSpeed_Motor_1(RT.get());
    Sub_Pruebas.setSpeed_Motor_2(LT.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Sub_Pruebas.setSpeed_Motor_1(0);
    Sub_Pruebas.setSpeed_Motor_2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
