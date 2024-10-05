// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Telemetry;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera piCamera1 = new PhotonCamera("Pi_Camera");
  // PhotonCamera piCamera2 = new PhotonCamera("Pi_Camera2");

  int FoundID; // Called in CheckID() pls dont call anywhere else
  int SelectedID;
  int Cycle = 0;
  private final Telemetry Tele = new Telemetry(0);
  
  public static SendableChooser<Integer> AprilTagSelector;
  private int lastCheckedTagId = -1; // Keeps track of the last Tag ID checked
  private boolean warningDisplayed = false; // Flag to track if warning has been shown
  private boolean Targetseen = false;
  
 
  //----------------------------------------------------------------simulator------------------------------------------------
  VisionSystemSim visionSim;
  SimCameraProperties cameraProp;
  public VisionSubsystem() {
    super();
    if (Robot.isSimulation()) {
      System.out.println("Sim Started");
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(getTagLayout());
      cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(32);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);
      System.out.println("Cam Set Up Comp");
      PhotonCameraSim cameraSim = new PhotonCameraSim(piCamera1, cameraProp);
      // X is forward and back and Y is Left and right and Z is Up and Down This is at floor level cause Z=0
      Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
      // 15 Degrees up
      Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
      Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
      visionSim.addCamera(cameraSim, robotToCamera);
      System.out.println("Vision Sim ready");
    }



    AprilTagSelector = new SendableChooser<Integer>();
    AprilTagSelector.setDefaultOption("Tag 1", 1);
    AprilTagSelector.addOption("Tag 2", 2);
    AprilTagSelector.addOption("Tag 3", 3);
    AprilTagSelector.addOption("Tag 4", 4);
    AprilTagSelector.addOption("Tag 5", 5);
    AprilTagSelector.addOption("Tag 6", 6);
    AprilTagSelector.addOption("Tag 7", 7);
    AprilTagSelector.addOption("Tag 8", 8);
    AprilTagSelector.addOption("Tag 9", 9);
    AprilTagSelector.addOption("Tag 10", 10);
    AprilTagSelector.addOption("Tag 11", 11);
    AprilTagSelector.addOption("Tag 12", 12);
    AprilTagSelector.addOption("Tag 13", 13);
    AprilTagSelector.addOption("Tag 14", 14);
    
    SmartDashboard.putData("AprilTag Selection", AprilTagSelector);

    var result = piCamera1.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      int StartupTargetID = target.getFiducialId();
      System.out.print("Target Found:");
      System.out.println(StartupTargetID);
      SmartDashboard.putNumber("April Tag Found", StartupTargetID);
    } else {
      System.out.println("Warning No Tag Found");
    }

    SmartDashboard.putBoolean("Found Tag?", CheckTagID(SelectedID));
  }

  public void simulationPeriodic() {
      
      Pose2d currentPose = Tele.getCurrentPose();
      visionSim.update(Tele.getCurrentPose());
      visionSim.getDebugField();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // GetResults(2);

    if (Cycle >= 50) {
      Cycle = 0;
      SmartDashboard.putBoolean("Found Tag?", CheckTagID(SelectedID));
    }
    
    Cycle = Cycle + 1;
    if(AprilTagSelector.getSelected().equals(1)) {
      SelectedID = 1;
    }
    else if (AprilTagSelector.getSelected().equals(2)){
      SelectedID = 2;
    }
    else if (AprilTagSelector.getSelected().equals(3)){
      SelectedID = 3;
    }
    else if (AprilTagSelector.getSelected().equals(4)){
      SelectedID = 4;
    }
    else if (AprilTagSelector.getSelected().equals(5)){
      SelectedID = 5;
    }
    else if (AprilTagSelector.getSelected().equals(6)){
      SelectedID = 6;
    }
    else if (AprilTagSelector.getSelected().equals(7)){
      SelectedID = 7;
    }
    else if (AprilTagSelector.getSelected().equals(8)){
      SelectedID = 8;
    }
    else if (AprilTagSelector.getSelected().equals(9)){
      SelectedID = 9;
    }
    else if (AprilTagSelector.getSelected().equals(10)){
      SelectedID = 10;
    }
    else if (AprilTagSelector.getSelected().equals(11)){
      SelectedID = 11;
    }
    else if (AprilTagSelector.getSelected().equals(12)){
      SelectedID = 12;
    }
    else if (AprilTagSelector.getSelected().equals(13)){
      SelectedID = 13;
    }
    else if (AprilTagSelector.getSelected().equals(14)){
      SelectedID = 14;
    }

   

  }



  

  // Declare these as class-level variables to persist across function calls


  private boolean CheckTagID(int TagId) {
    var result = piCamera1.getLatestResult();
    int FoundID = -1; 
    int CurrentID = -1; 
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        FoundID = target.getFiducialId(); 
        CurrentID = FoundID;
        if (FoundID == TagId) {
            lastCheckedTagId = TagId; 
            warningDisplayed = false;
            return true; 
        }
    }
    if (FoundID != TagId) {
        if (lastCheckedTagId != TagId) {
            lastCheckedTagId = TagId;
            warningDisplayed = false; 
        }
        if (!warningDisplayed) {
            System.out.print("Warning April Tag: ");
            System.out.print(TagId);
            System.out.println(" Not found");
            warningDisplayed = true; 
        }
        return false; 
    }
    return false; 
  }

  public double FindTagIDyaw(int TagId) {
    var result = piCamera1.getLatestResult();
    double targetYaw = 0.0;

    if (result.hasTargets()) {
      for (var target : result.getTargets()) {
          if (target.getFiducialId() == TagId) {
              targetYaw = target.getYaw();
              
          }
      }
    } else {
      targetYaw = 0.0;
    }
    return targetYaw;
  }






































  // private void GetResults(double x){   WIP
  //   if (x == 1) {
  //     var result = piCamera1.getLatestResult();
  //   } else if (x == 2) {
  //     var result = piCamera2.getLatestResult();
  //   }
    
  //   boolean hasTargets = result.hasTargets();
  // }
  
}
