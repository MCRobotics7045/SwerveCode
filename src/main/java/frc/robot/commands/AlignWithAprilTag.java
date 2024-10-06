// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;



// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import static frc.robot.Constants.Vision.*;

// import frc.robot.Constants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.VisionSubsystem;

// public class AlignWithAprilTag extends Command {
//   /** Creates a new AlignWithAprilTag. */
//   private final SwerveRequest.ApplyChassisSpeeds drivetrain = new SwerveRequest.ApplyChassisSpeeds();

//   private final CommandSwerveDrivetrain swerve;
//   private final Translation2d
//   public AlignWithAprilTag(CommandSwerveDrivetrain swerve) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.swerve = swerve;
//     addRequirements(swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     swerve.applyRequest(drivetrain.CenterOfRotation(0,0))
     
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
  
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
    
   
//       return true;


 
    
//   }
// }
