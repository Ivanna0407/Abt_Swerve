// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Cmd_Auto_Move_Intake;
import frc.robot.commands.Cmd_Auto_Move_Shooter;
import frc.robot.commands.Cmd_Intake_shoot;
import frc.robot.commands.Cmd_Move_Swerve;
import frc.robot.commands.Cmd_Specific_State;
import frc.robot.commands.Cmd_Take;
import frc.robot.commands.Cmd_Wait;
import frc.robot.commands.Cmd_Zero_Heading;
import frc.robot.subsystems.Sub_Intake_Shooter;
import frc.robot.subsystems.Sub_Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  private final Sub_Swerve Swerve= new Sub_Swerve();
  private final Sub_Intake_Shooter Intake_Shooter = new Sub_Intake_Shooter();
  
  CommandXboxController Joydrive= new CommandXboxController(0);
  CommandXboxController Subdrive= new CommandXboxController(1);
 
  public RobotContainer() {
    Swerve.setDefaultCommand(new Cmd_Move_Swerve(Swerve,() -> Joydrive.getLeftX(),() -> Joydrive.getLeftY(), () -> Joydrive.getRightX(), ()-> Joydrive.x().getAsBoolean()));
    Intake_Shooter.setDefaultCommand(new Cmd_Intake_shoot(Intake_Shooter, () -> Subdrive.x().getAsBoolean(), () -> Subdrive.y().getAsBoolean(), () -> Subdrive.getLeftTriggerAxis(), () -> Subdrive.b().getAsBoolean(), () -> Subdrive.rightBumper().getAsBoolean()));
    //Swerve.setDefaultCommand(new Cmd_Specific_State(Swerve));

    configureBindings();
  }


  private void configureBindings() {
    Joydrive.start().whileTrue(new Cmd_Zero_Heading(Swerve));
    Joydrive.b().whileTrue(new Cmd_Specific_State(Swerve));
    Subdrive.a().whileTrue(new Cmd_Take(Intake_Shooter));
  }


  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(new Cmd_Auto_Move_Shooter(Intake_Shooter, -1),
      new Cmd_Wait(1),
      new Cmd_Auto_Move_Intake(Intake_Shooter, .6),
      new Cmd_Wait(0.5),
      new Cmd_Auto_Move_Shooter(Intake_Shooter, 0),
      new Cmd_Auto_Move_Intake(Intake_Shooter, 0));
  }
  
  //return null;
  }

