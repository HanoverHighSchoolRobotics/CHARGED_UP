// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Slide;

public class SlideWithXbox extends CommandBase {

  XboxController xbox;

  XboxController xbox1;
  XboxController xbox2;

  Slide slide;

  enum AngleState {

    AUTOMATIC_CONTROL,
    MANUAL_CONTROL

  }
 
  double targetAngle = 0;

  AngleState currentAngleState = AngleState.AUTOMATIC_CONTROL;

  public SlideWithXbox(XboxController xb1, XboxController xb2, Slide sl) {
  
    xbox1 = xb1;
    xbox2 = xb2;
    slide = sl;

    addRequirements(slide);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (Constants.modeSelect.getSelected() == "Player_Two") {

      xbox = xbox1;
      
    } else {

      xbox = xbox2;

    }
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if (xbox.getLeftY() > .2 || xbox.getLeftY() < -.2) {

      currentAngleState = AngleState.MANUAL_CONTROL;

    } else {

      currentAngleState = AngleState.AUTOMATIC_CONTROL;

    }

    if (currentAngleState == AngleState.MANUAL_CONTROL) {

      slide.changeAngleUsingPower(-xbox.getLeftY() / 1);
      targetAngle = slide.returnAngle();

    }

    if (currentAngleState == AngleState.AUTOMATIC_CONTROL) {

      if (slide.returnAngle() < targetAngle - .5) {

        slide.changeAngleUsingPower(-.3);

      } else if (slide.returnAngle() > targetAngle + .5) {

        slide.changeAngleUsingPower(.3);

      } else {

        slide.changeAngleUsingPower(0);

      }

    }

    if (xbox.getYButtonPressed()) {

      targetAngle = 15;

    }

    if (xbox.getXButtonPressed()) {

      targetAngle = -15;

    }

    if (xbox.getStartButtonPressed()) {

      targetAngle = 0;

    }

    if (xbox.getLeftTriggerAxis() > .5) {

      targetAngle = SmartDashboard.getNumber("Custom Angle", 0);

    }
 
    slide.extendArmUsingPower(-xbox.getRightY() / 1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    slide.changeAngleUsingPower(0);
    slide.extendArmUsingPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
