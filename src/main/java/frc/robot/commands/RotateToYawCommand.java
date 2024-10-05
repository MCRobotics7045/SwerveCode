// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class RotateToYawCommand extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final Pigeon2 pigeon;
  private final double targetYaw;
  private static final double MAX_TURN_RATE = 1.0;
  private static final double ERROR_MARGIN = 2.0;    
  private static final double kP = 0.05;             
  private static final double DEAD_BAND = 1.0;       
  private final SwerveRequest.FieldCentric fieldCentricRequest;

  public RotateToYawCommand(CommandSwerveDrivetrain swerve, Pigeon2 pigon, double targetYaw) {
    this.swerve = swerve;
    this.pigeon = pigon;
    this.targetYaw = targetYaw;
    this.fieldCentricRequest = new SwerveRequest.FieldCentric();
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Rotate to Yaw Command initialized. Target Yaw: " + targetYaw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentYaw = pigeon.getAngle();
    double yawDifference = ((targetYaw - currentYaw + 540)% 360) - 180;
    System.out.println("Current Yaw: " + currentYaw);
    System.out.println("Yaw Difference: " + yawDifference);
    double turnRate = yawDifference * kP;
    turnRate = Math.max(-MAX_TURN_RATE, Math.min(turnRate, MAX_TURN_RATE));
    if (Math.abs(yawDifference) < DEAD_BAND) {
      turnRate = 0;
    }
    System.out.println("Calculated Turn Rate: " + turnRate);



    fieldCentricRequest.withRotationalRate(turnRate);

    swerve.setDefaultCommand(swerve.applyRequest(() -> fieldCentricRequest));
    System.out.println("Turn Rate Applied: " + turnRate);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fieldCentricRequest.withRotationalRate(0);
    swerve.applyRequest(() -> fieldCentricRequest);
    System.out.println("Rotate to Yaw Command ended: " + (interrupted ? "Interrupted" : "Completed"));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentYaw = pigeon.getAngle();  
    double yawDifference = ((targetYaw - currentYaw + 540) % 360) - 180;
    System.out.println("Current Yaw for Finish Check: " + currentYaw);
    return Math.abs(yawDifference) < ERROR_MARGIN;  
  }
}
