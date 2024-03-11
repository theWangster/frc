// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Arm {

  CANSparkBase m_launchWheel;
  CANSparkBase m_feedWheel;
  CANSparkBase m_rollerClaw;
  CANSparkBase m_climber;
  Joystick Controller;
  boolean isRamping = false;
  double curTime = Timer.getFPGATimestamp();
  double launchTime = -1;

  public void armInit(Joystick Controller) {
    m_feedWheel = new CANSparkMax(6, MotorType.kBrushed);
    m_launchWheel = new CANSparkMax(5, MotorType.kBrushed);
    this.Controller = Controller;
    // To change launcher direction
    m_feedWheel.setInverted(true);
    m_launchWheel.setInverted(true);
    // Applies Amps limits to launcher
    m_feedWheel.setSmartCurrentLimit(Constants.FEEDER_CURRENT_LIMIT_A);
    m_launchWheel.setSmartCurrentLimit(Constants.LAUNCHER_CURRENT_LIMIT_A);
  }


  public void shoot() {
    curTime = Timer.getFPGATimestamp();

    if (Controller.getRawButtonPressed(1)) {
      isRamping = true;
      m_launchWheel.set(1);
      m_feedWheel.set(1);
    } else if (Controller.getRawButtonReleased(1)) {
      // this is disgraceful but ok
      if (false) {
        m_feedWheel.set(1);
        m_launchWheel.set(1);
      } else {
        m_feedWheel.set(0);
        m_launchWheel.set(0);
      }
    }

    // intake code
    if (Controller.getRawButtonPressed(4)) {
      m_feedWheel.set(-0.50);
      m_launchWheel.set(-0.50);
    } else if (Controller.getRawButtonReleased(4)) {
      m_feedWheel.set(0);
      m_launchWheel.set(0);
    }
  }

  // While the button is being held spin both motors to intake note
  // if (Controller.getRawButton(Constants.Intake_Button)) {
  // 	m_launchWheel.set(-LAUNCHER_SPEED);
  // 	m_feedWheel.set(Constants.FEEDER_IN_SPEED);
  // } else if (Controller.getRawButtonReleased(5)) {
  // 	m_launchWheel.set(0);
  // 	m_feedWheel.set(0);
  // }

  // While the amp button is being held, spin both motors to "spit" the note
  // out at a lower speed into the amp
  // if (Controller.getRawButton(Constants.Amp_Button)) {
  // 	m_feedWheel.set(Constants.FEEDER_AMP_SPEED);
  // 	m_launchWheel.set(LAUNCHER_AMP_SPEED);
  // } else if (Controller.getRawButtonReleased(Constants.Amp_Button)) {
  // 	m_feedWheel.set(0);
  // 	m_launchWheel.set(0);
  // }
}
