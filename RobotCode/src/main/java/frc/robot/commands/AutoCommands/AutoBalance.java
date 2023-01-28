// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private Drivetrain drivetrain = new Drivetrain();
  private XboxController xbox;
  private double unbalancedAngleForward = 5;
  private double unbalancedAngleBack = -5;
  
  private double speedForward = .25;
  private double speedBack = -.25;
  private double speedNone = 0;

  private String movementDirection = "Level";


  public AutoBalance(Drivetrain dt, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    xbox = xboxController;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Robot Speed Forwards", speedForward);
    SmartDashboard.putNumber("Robot Speed Backwards", speedBack);
    //SmartDashboard.putNumber("Robot Angle", drivetrain.getNavXPitchOutput()); moved to DriveWithXbox
    SmartDashboard.putString("Is robot level", movementDirection);

    speedBack = drivetrain.getNavXPitchOutput() * Constants.aBalanceValue;
    speedForward = drivetrain.getNavXPitchOutput() * Constants.aBalanceValue;

    if (speedBack < -1){
      speedBack = -1;
    } else if (speedBack > 0){
      speedBack = 0;
    }
    
    if (speedForward > 1){
      speedForward = 1;
    } else if (speedForward < 0){
      speedForward = 0;
    }

    //When button is held locks all wheels forward
    //Move forward or backwards according to angle
    //Repeats until you let go of the button       

    //IMPORTANT
    //Make a function for the robot to use the NaxV yaw to make its heading perpendicular to the ramp.
    //NavX Yaw makes robot rotate until its perpendiculor to ramp.
    if (xbox.getXButton()){

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 0, 1);

        //When navx thinks tilted back, drive motors forward
        if (drivetrain.getNavXPitchOutput() > unbalancedAngleForward){
          drivetrain.driveAllModules(speedForward);
          movementDirection = "Tilted Backwards";
        
        //When navx thinks tilted forward, drive motors backwards
        } else if (drivetrain.getNavXPitchOutput() < unbalancedAngleBack){
          drivetrain.driveAllModules(speedBack);
          movementDirection = "Tilted Forwards";
   
        } else {

          drivetrain.driveAllModules(speedNone);
          movementDirection = "Level";       

        }
      }
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xbox.getXButton() == false) {
        return true;
    } else {
      return false;
    }
  }
}
