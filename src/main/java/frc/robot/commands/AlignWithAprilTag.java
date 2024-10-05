// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Constants.Vision.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithAprilTag extends Command {
  /** Creates a new AlignWithAprilTag. */
  private VisionSubsystem Vision;
  private CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
  private Pigeon2 Pigon;
  private final SwerveRequest.FieldCentric drive;
  private double turnRate;
  private final int TagID;
  private double targetYaw;
  private double currentYaw;
  private double yawDifference;

  private static final double MAX_TURN_RATE = 5.0; 
  private static final double ERROR_MARGIN = 2.0; 
  private static final double DEAD_BAND = 1.0; 
  public AlignWithAprilTag(CommandSwerveDrivetrain m_swerve,int m_TagID,SwerveRequest.FieldCentric drive,VisionSubsystem Vision, Pigeon2 Pigon) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;
    TagID = m_TagID;
    this.drive = drive;
    this.Vision = Vision;
    this.Pigon = Pigon;
    addRequirements(swerve, Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Align with AprilTag Init, Tag Id: "+ TagID);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetYaw = Vision.FindTagIDyaw(TagID); 
    currentYaw = Pigon.getAngle(); 
    yawDifference = ((targetYaw - currentYaw + 540) % 360) - 180; 

    System.out.println("Current Yaw: " + currentYaw);
    System.out.println("Target Yaw: " + targetYaw);
    System.out.println("Yaw Difference: " + yawDifference);

    turnRate = -yawDifference * VISION_TURN_kP; 
    turnRate = Math.max(-MAX_TURN_RATE, Math.min(turnRate, MAX_TURN_RATE)); 

    if (Math.abs(yawDifference) < DEAD_BAND) {
      turnRate = 0; // Stop turning if within deadband
    }

    System.out.println("Calculated Turn Rate: " + turnRate);

    swerve.setDefaultCommand( 
      swerve.applyRequest(() -> drive .withRotationalRate(turnRate)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.applyRequest(() -> drive.withRotationalRate(0)); // Stop the robot
    System.out.println("Align with AprilTag ended: " + (interrupted ? "Interrupted" : "Completed"));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double targetYaw = Vision.FindTagIDyaw(TagID); 
    double currentYaw = Pigon.getAngle(); 
    double yawDifference = ((targetYaw - currentYaw + 540) % 360) - 180; 

    boolean aligned = Math.abs(yawDifference) < ERROR_MARGIN;

    System.out.println("Current Yaw for Finish Check: " + currentYaw);
    return aligned; // Finish if aligned
  }
}
