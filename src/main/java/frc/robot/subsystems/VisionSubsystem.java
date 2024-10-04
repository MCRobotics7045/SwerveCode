// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.text.html.HTML.Tag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera piCamera1 = new PhotonCamera("Pi_Camera");
  // PhotonCamera piCamera2 = new PhotonCamera("Pi_Camera2");

  int FoundID; // Called in CheckID() pls dont call anywhere else
  int SelectedID;
  public static SendableChooser<Integer> AprilTagSelector;
  private int lastCheckedTagId = -1; // Keeps track of the last Tag ID checked
  private boolean warningDisplayed = false; // Flag to track if warning has been shown

  public VisionSubsystem() {
    super();


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


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // GetResults(2);

    
    

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
   SmartDashboard.putBoolean("Found Tag?", CheckTagID(SelectedID));

  }



  // Declare these as class-level variables to persist across function calls


private boolean CheckTagID(int TagId) {
    var result = piCamera1.getLatestResult();
    int FoundID = -1; // Initialize FoundID to -1 (no ID found initially)
    int CurrentID = -1; // Initialize CurrentID to -1 for cases where no targets are found

    // Check if the camera result has targets
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        FoundID = target.getFiducialId(); // Get the ID of the best target
        CurrentID = FoundID; // Set the current ID to the found ID

        // Reset the warning if a target is found and it's the desired tag
        if (FoundID == TagId) {
            lastCheckedTagId = TagId; // Update last checked ID to the current one
            warningDisplayed = false; // Reset the warning since the tag is now found
            return true; // Return true since the correct tag is found
        }
    }

    // If no target is found or the correct tag isn't found
    if (FoundID != TagId) {
        // Check if we're still checking the same TagId and whether the warning has been displayed
        if (lastCheckedTagId != TagId) {
            // If a new TagId is being checked, reset the warning system
            lastCheckedTagId = TagId; // Update the last checked ID to the new one
            warningDisplayed = false; // Reset the warning display for the new tag
        }

        // Display the warning only if it hasn't been shown yet
        if (!warningDisplayed) {
            System.out.print("Warning April Tag: ");
            System.out.print(TagId);
            System.out.println(" Not found");
            warningDisplayed = true; // Mark that the warning has been shown
        }

        return false; // Return false as the correct tag is not found
    }

    return false; // If no tag is found at all, return false
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
