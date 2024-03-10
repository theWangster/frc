package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class PID {

    public double KP;
    public double KD;
    public double KI;
    public double Int; // Integral iterates.
    public double PrevError; // Previous error for each motor
    public double PrevTime; // Previous time since each motor specific PID ran

    public PID(double KP, double KI, double KD){
        // PID Global Coefficient Arrays {MotorA, MotorB, MotorC, MotorD}:
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        // Integral, PrevTime and PrevError have to be made as arrays to be called by index parameter if accessed by multiple motors.
        Int = 0; // Integral array iterates.
        PrevError = 0; // Previous error for each motor
        PrevTime = Timer.getFPGATimestamp(); // PrevTime should be set to current time before intitally running PID loop, then set to the last time the PID ran...should PrevTime be an array as well for the last time each motor ran?
    }


    public void reset(){
        Int = 0; // Integral array iterates.
        PrevError = 0; // Previous error for each motor
        PrevTime = Timer.getFPGATimestamp(); // PrevTime should be set to current time before intitally running PID loop, then set to the last time the PID ran...should PrevTime be an array as well for the last time each motor ran?
    }


    public double runPID(double input, double target) { // PID is for the speed of the motors; put in motorAccel function at every incrementation of speed. Each motor has a target speed, input and index.    
        // Error:
        double Error = target - input; // EncoderRPM should be divided by max rpm to ensure it is within -1 <= x <= 1 motor speed range.
        // Proportional Term:
        double Proportional = Error*KP;
        // Calculus Terms:
        double CurrTime = Timer.getFPGATimestamp();
        double dt = CurrTime-PrevTime; // delta time (differential of time) = Δt. Measured from the start of the first call to the calling, or the time in between callings.
        double Derivative = ((Error - PrevError) / dt) * KD; // Derivative is v'(t) = d/dt(v(t) = a(t), or Δv/Δt = a, dt is delta time.
        Int = Int + (Error*dt*KI); // Integral accumulates the error bars over time, and for every new movement Integral = 0. KI must be very small.
        double Integral = Int; // Integral increments the error by dt, and should have a KI that gradually decreases or is very low; Integral is set to 0 when a new PID loop is called.
        
        // Returned values and continuities:
        PrevError = Error;
        PrevTime = CurrTime;
        double ANS = (Proportional + Derivative + Integral);
        return ANS; // -1 <= ANS <= 1
    }
}
   /**
   * Example for Arrayed PID:
   *
   * // Inititializing before first run
   *  PrevError[] = 0.0;
   *  PrevTime[] = Timer.getFPGATimestamp();
   *  Integral[] = 0;
   *
   *  teleoploop() {
   *    Otherfunctions();
   *    MotorA = PID((EncoderARPM/MaxRPM), TargetMotorASpeed, indexA); // MaxRPM is a constant
   *    MotorB = PID((EncoderBRPM/MaxRPM), TargetMotorBSpeed, indexB);
   *    MotorC = PID((EncoderCRPM/MaxRPM), TargetMotorCSpeed, indexC);
   *    MotorD = PID((EncoderDRPM/MaxRPM), TargetMotorDSpeed, indexD);
   *    if(movement_change == True) {
   *      IntArr[] = 0.0; // If we set a array to = x, just loop through its indices.
   *      PrevError[] = 0.0;
   *      PrevTimeArr[] = Timer.getFPGATimestamp();
   *    }
   *  }
   *
   *
   *
   * Only if you have a sensitive system like a drone do you set a secant f(x) as = KI, such that it does not over-consider outliers as rhe integrator winds up. For big bot, lim(KI --> 0).
   */
