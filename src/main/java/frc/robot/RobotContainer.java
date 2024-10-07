// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Cmd_Move_Swerve;
import frc.robot.commands.Cmd_Zero_Heading;
import frc.robot.subsystems.Sub_Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  private final Sub_Swerve Swerve= new Sub_Swerve();
  
  CommandXboxController Joydrive= new CommandXboxController(0);
 
  public RobotContainer() {
    Swerve.setDefaultCommand(new Cmd_Move_Swerve(Swerve,() -> Joydrive.getLeftX(),() -> Joydrive.getLeftY(), () -> Joydrive.getRightX(), ()-> Joydrive.x().getAsBoolean()));
    configureBindings();
  }


  private void configureBindings() {
    Joydrive.start().whileTrue(new Cmd_Zero_Heading(Swerve));
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
