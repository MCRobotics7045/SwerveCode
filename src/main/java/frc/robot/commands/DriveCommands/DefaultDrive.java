// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import static frc.robot.Constants.Constants.SwerveConstants.*;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.io.InputStreamReader;
public class DefaultDrive extends Command {

  private XboxController XBOX;
  private SwerveSubsystem SWERVE;
  private double xVelocity;
  private double yVelocity;
  private double rotationalVelocity;

  SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SlewRate);
  SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SlewRate);


  public DefaultDrive(XboxController XBOX, SwerveSubsystem SWERVE) {
    this.SWERVE = SWERVE;
    this.XBOX = XBOX;
    addRequirements(SWERVE);
    Shuffleboard.getTab("help").add("X", xVelocity);
    Shuffleboard.getTab("help").add("Y", yVelocity);
    Shuffleboard.getTab("help").add("Rotational", rotationalVelocity);

  }

 
  @Override
  public void execute() {
    double InputX = MathUtil.applyDeadband(XBOX.getLeftX(), .1) * SwerveConstants.MaxSpeed;
    double InputY = MathUtil.applyDeadband(XBOX.getLeftY(), .1) * SwerveConstants.MaxSpeed;
    double InputZ = MathUtil.applyDeadband(XBOX.getRightX(), .15) * SwerveConstants.MaxRotationSpeed;
    yVelocity = InputY;
    xVelocity = InputX;
    rotationalVelocity = InputZ;

    xVelocity = xVelocityFilter.calculate(InputX);
    yVelocity = yVelocityFilter.calculate(InputY);
    // InputX = MathUtil.applyDeadband(InputX, .1) * 12;
    // InputY = MathUtil.applyDeadband(InputY, .1) * 12;
    // InputZ = MathUtil.applyDeadband(InputZ, .15);
    // double forwardDirection = (RobotContainer.IsRed() ? 1.0 : -1.0);
    // double inputDir = Math.atan2(InputY, InputX);
    // double inputMagnitude = Math.hypot(InputX, InputY);
		//xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    // yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * MaxSpeed * forwardDirection * SWERVE.SpeedMultipler);
    // rotationalVelocity = (InputZ * angularSpeed );
    // rotationalVelocity = MathUtil.applyDeadband(rotationalVelocity, rotationalVelocity);


    SWERVE.drive(yVelocity, xVelocity, rotationalVelocity);
    
    System.out.println(xVelocity + "  " + yVelocity + "   " + rotationalVelocity);
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
