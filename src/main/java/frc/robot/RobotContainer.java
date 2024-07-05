// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  // The robot's subsystems and commands are defined here...
  private final Sub_Swerve Swerve= new Sub_Swerve();
 
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController Joydrive= new CommandXboxController(0);
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    Swerve.setDefaultCommand(new Cmd_Move_Swerve(Swerve,() -> Joydrive.getLeftX(),() -> Joydrive.getLeftY(), () -> Joydrive.getRightX(), ()-> Joydrive.x().getAsBoolean()));
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //Joydrive.start().whileTrue(new Cmd_Zero_Heading(Swerve));
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
