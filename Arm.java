// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Arm {
    CANSparkBase m_launchWheel;
	CANSparkBase m_feedWheel;
	CANSparkBase m_rollerClaw;
	CANSparkBase m_climber;
	Joystick Controller;
	public void armInit(Joystick Controller){
		m_feedWheel = new CANSparkMax(5, MotorType.kBrushed);
		m_launchWheel = new CANSparkMax(6, MotorType.kBrushed);
		this.Controller = Controller;
	

		// To change launcher direction
		m_feedWheel.setInverted(true);
		m_launchWheel.setInverted(true);

		// Applies Amps limits to launcher
		m_feedWheel.setSmartCurrentLimit(Constants.FEEDER_CURRENT_LIMIT_A);
		m_launchWheel.setSmartCurrentLimit(Constants.LAUNCHER_CURRENT_LIMIT_A);
	}

	public void shoot() {
		// Arm and Other controls

			// if (Controller.getRawButton(Constan ts.Launcher_Button)) {
			// 	m_launchWheel.set(LAUNCHER_SPEED);
			// } else if (Controller.getRawButtonReleased(1)) {
			// 	m_launchWheel.set(0);
			// }

			// Spins feeder wheel, wait for launch wheel to spin up to full speed for best
			// results
	
			if (Controller.getRawButtonPressed(1)) {
				// double initialtime = Timer.getFPGATimestamp();
				// System.out.println("Initial Time:"+initialtime);
				// double finaltime = initialtime + 3;
				// System.out.println("final time:"+finaltime);
				// System.out.println("Time reached");
				m_feedWheel.set(1.2);
				m_launchWheel.set(1.2);
			}else if(Controller.getRawButtonReleased(1)){
					m_feedWheel.set(0);
					m_launchWheel.set(0);
				}
			if (Controller.getRawButtonPressed(4)){
				// double initialtime = Timer.getFPGATimestamp();
				// System.out.println("Initial Time:"+initialtime);
				// double finaltime = initialtime + 3;
				// System.out.println("final time:"+finaltime);
				// System.out.println("Time reached");
				m_feedWheel.set(-0.50);
				m_launchWheel.set(-0.50);
			}else if(Controller.getRawButtonReleased(4)){
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

	

