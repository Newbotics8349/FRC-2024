package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.AprilTagTracking.*;

import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.annotation.JsonTypeInfo.None;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class Arm {
     //functional motors
  public CANSparkMax armMotor1;
  public CANSparkMax armMotor2; 
  private CANSparkMax intakeMotor;
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  public double ampPosition;
  private double angleToEncoderPosition;
  private final double starterAngle = 0;
  private DigitalInput proximitySensor;
  public double maxArmAngle;
  public double minArmAngle;
   
  
    public Arm() {
        //functional motors
        armMotor1 = new CANSparkMax(1, MotorType.kBrushless); // left arm base
        armMotor2 = new CANSparkMax(2, MotorType.kBrushless); //right arm base
        intakeMotor = new CANSparkMax(3, MotorType.kBrushed); // pwm set
        shooterMotor1 = new CANSparkMax(4, MotorType.kBrushless); //left shooter
        shooterMotor2 = new CANSparkMax(9, MotorType.kBrushless); // right shooter
        proximitySensor = new DigitalInput(1); // Use the actual DIO port number

    }
    public boolean checkSensorandNotify() { // method to return whether a note is loaded or not
        boolean noNoteDetected = proximitySensor.get(); // This will return true if nothing is detected 
    // Assuming the sensor output is HIGH when an object is detected
    if (noNoteDetected) { // When no note is detected, this if statement occurs
    SmartDashboard.putString("Alert", "No note in the intake.");}
    // turn LED colour
    else{ // When note is detected, this else statement occurs
    SmartDashboard.putString("Alert", "Note is in the intake!");}
    return noNoteDetected;
    }

    public double getArmAngle() {
    double currentEncoderPosition = armMotor1.getEncoder().getPosition();
    double currentAngle = currentEncoderPosition * angleToEncoderPosition + starterAngle;
    return currentAngle;
  }
  // shooter
  public void shooter(double power) {
    intakeMotor.set(power);
    shooterMotor1.set(power);
    shooterMotor2.set(power);
      //eventually reset motors to zero
  }
  // intake
  public void intake(double power) {
    //enableDrive = false;
    // if there is no note the intake motor runs
    while (checkSensorandNotify() == true){
    intakeMotor.set(power);
    //once the note is in the intake the motor stops
    if (checkSensorandNotify() == false) {
    intakeMotor.set(0); }
    //enableDrive = true;
  }}

    public void setMotorPower (double power) {
        final double maxPower = 0.5;
        if (power > maxPower) {
            power = maxPower;
        }
        else if (power < -maxPower) {
            power = -maxPower;
        }
        armMotor1.set(power);
        armMotor2.set(power);
    }

    public void moveToPosition(double angle) {
        if (angle > maxArmAngle) {
            angle = maxArmAngle;
        }
        else if (angle < minArmAngle) {
            angle = minArmAngle;
        }
        double Kp = 0; // change to actual after testing*************
        double error = angle - getArmAngle(); // current position (after converting)*****
        double power = Kp * error;
        setMotorPower(power);


    }
}
