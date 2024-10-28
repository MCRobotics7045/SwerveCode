// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.Constants.IntakeSubsytem.*;
import frc.robot.subsystems.VisionSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  public CANSparkMax IntakeIndexer = new CANSparkMax(IntakeIndexerId, MotorType.kBrushless);
  private Mechanism2d IntakeModel = new Mechanism2d(2, 2);
  private MechanismRoot2d root = IntakeModel.getRoot("Intake", 1, .2);
  private final MechanismLigament2d m_intakeBase;
  private final MechanismLigament2d m_intakeWristT;
  private final MechanismLigament2d m_intakeWristB;

  public IntakeSub() {
    super();
    IntakeIndexer.restoreFactoryDefaults();
    m_intakeBase = root.append(new MechanismLigament2d("Base", .5, 180, 8, new Color8Bit(Color.kAliceBlue) ));
    m_intakeWristT = m_intakeBase.append(new MechanismLigament2d("Top", .2, 90, 6,new Color8Bit(Color.kWhite)));
    m_intakeWristB = m_intakeBase.append(new MechanismLigament2d("Bottom", -.2, 90, 6,new Color8Bit(Color.kWhite)));
    // m_intakeBase.setLength(.5);

    if (Robot.isSimulation()) {
      System.out.println("Intake Started");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      SmartDashboard.putData("Intake Mech", IntakeModel);

  }
  public void Fire(double Angle){

    double CorrectedAngle = Math.toDegrees(Angle);
    m_intakeBase.setAngle(CorrectedAngle/2);
    m_intakeWristT.setAngle(CorrectedAngle);
    m_intakeWristB.setAngle(CorrectedAngle);
  }
}
