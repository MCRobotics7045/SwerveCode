// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



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
  private final SwerveRequest.FieldCentric drive;
  private double turn;
  private int TagID;
  
  
  public AlignWithAprilTag(CommandSwerveDrivetrain m_swerve,int m_TagID,SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;
    TagID = m_TagID;
    this.drive = drive;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Align with AprilTag Init, Tag Id: ");
    System.out.println(TagID);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turn = -1.0 * Vision.FindTagIDyaw(TagID) * VISION_TURN_kP * kMaxAngularSpeed;
    
    swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive 
        .withRotationalRate(turn) // Drive counterclockwise with negative X (left)
        ));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Vision.FindTagIDyaw(TagID) >= ErrorMarginNegative && Vision.FindTagIDyaw(TagID) <= ErrorMarginPostive) {
      return true;
    } else {
      return false;
    }
    
  }
}
