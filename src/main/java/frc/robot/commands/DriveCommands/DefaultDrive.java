// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import static frc.robot.Constants.Constants.SwerveConstants.*;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
public class DefaultDrive extends Command {

  private XboxController XBOX;
  private SwerveSubsystem SWERVE;
  SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SlewRate);


  public DefaultDrive(XboxController XBOX, SwerveSubsystem SWERVE) {
    this.SWERVE = SWERVE;
    this.XBOX = XBOX;
    addRequirements(SWERVE);
  }

 
  @Override
  public void execute() {
    double InputX = XBOX.getLeftX();
    double InputY = XBOX.getLeftY();
    double InputZ = XBOX.getRightX();
    InputX = MathUtil.applyDeadband(InputX, .1);
    InputY = MathUtil.applyDeadband(InputY, .1);
    double forwardDirection = (RobotContainer.IsRed() ? -1.0 : 1.0);
    double inputDir = Math.atan2(InputY, InputX);
    double inputMagnitude = Math.hypot(InputX, InputY);
		double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    double rotationalVelocity = (InputZ * angularSpeed );
    SWERVE.drive(yVelocity, xVelocity, rotationalVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}