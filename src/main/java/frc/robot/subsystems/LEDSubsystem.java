// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {


  private AddressableLED Strip1;
  private AddressableLEDBuffer Strip1Buffer;

  public LEDSubsystem() {
    Strip1 = new AddressableLED(Strip1PWM);
    
  }

  @Override
  public void periodic() {
    
  }
}
