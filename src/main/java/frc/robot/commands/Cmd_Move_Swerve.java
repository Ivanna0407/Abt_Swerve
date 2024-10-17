// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Sub_Swerve;
import java.util.function.Supplier;

public class Cmd_Move_Swerve extends Command {
  private final Sub_Swerve sub_Swerve;
  private final Supplier<Double>  Xaxis,Yaxis,giros;
  private final Supplier<Boolean>fieldoriented;
  public Cmd_Move_Swerve(Sub_Swerve Sub_Swerve,Supplier<Double> Xaxis,Supplier<Double> Yaxis,Supplier<Double> giros,Supplier<Boolean> fieldoriented) {
    this.sub_Swerve=Sub_Swerve;
    this.Xaxis=Xaxis;
    this.Yaxis=Yaxis;
    this.giros=giros;
    this.fieldoriented=fieldoriented;
    addRequirements(Sub_Swerve);
  }

 
  @Override
  public void initialize() {
    
    sub_Swerve.zeroHeading();
  }


  @Override
  public void execute() {
    double velocidadx=Xaxis.get();
    double velocidady=Yaxis.get()*-1;
    double velocidad_giros=giros.get();

    if (Math.abs(Xaxis.get())<0.1){velocidadx=0;}
    if (Math.abs(Yaxis.get())<0.1){velocidady=0;}
    if (Math.abs(giros.get())<0.1){velocidad_giros=0;}

    ChassisSpeeds chassisSpeeds;
    if (fieldoriented.get()){
      chassisSpeeds= ChassisSpeeds.fromFieldRelativeSpeeds(velocidady, velocidadx, velocidad_giros, sub_Swerve.get2Drotation());
    }
    else{
      chassisSpeeds= new ChassisSpeeds(0, 1, 0); //Es muy importante recordar que primero es y alias frente
      //luego x que es derecha e izquierda y finalmente los giros sobre el eje 
      
      
    }
    

    //Manda un arreglo de estados de modulos que pasan por un objeto de Swerve drive kinematics para poder generar las velocidades
    SwerveModuleState[] moduleStates=Swerve.swervekinematics.toSwerveModuleStates(chassisSpeeds);
    sub_Swerve.setModuleStates(moduleStates);
  }


  @Override
  public void end(boolean interrupted) {
    sub_Swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
