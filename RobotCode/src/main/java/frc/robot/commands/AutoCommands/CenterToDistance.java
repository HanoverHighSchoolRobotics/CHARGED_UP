// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class CenterToDistance extends CommandBase {
  
  Drivetrain drivetrain;
  Limelight limelight;
  double targetDistance;
  double speed;
  double tolerance;
  double stage = 1;
  Timer rotateTimer;

  public CenterToDistance(Drivetrain dt, Limelight lm, double TargetDistance, double Speed, double Tolerance) {
    
    drivetrain = dt;
    limelight = lm;
    targetDistance = TargetDistance;
    speed = Speed;
    tolerance = Tolerance;

    addRequirements(drivetrain, limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotateTimer = new Timer();
    drivetrain.resetOdometryToLimelight();
    rotateTimer.reset();
    rotateTimer.start();
    stage = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("In Loop");
 
    if (stage == 1) {

      System.out.println("Stage 1");

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 90, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 90, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 90, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 90, 1);

      if (rotateTimer.get() >= 1) {

        stage = 2;

      }
 
    }

    if (stage == 2) {

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 90, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 90, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 90, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 90, 1);

      if (drivetrain.getOdometryY() > targetDistance + tolerance) {

        drivetrain.driveAllModules(-speed);

      } else if (drivetrain.getOdometryY() > targetDistance - tolerance) {

        drivetrain.driveAllModules(speed);

      } else {

        drivetrain.driveAllModules(0);
        end(false);

      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    stage = 1;
    drivetrain.driveAllModules(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
   
  }
}
