// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// hello whats up

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
import frc.robot.Arm;

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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // controls
  private Joystick joystick; //joystick
  private Joystick joystick2; //joystick2

  //accelerometer
  private BuiltInAccelerometer builtInAccelerometer;
  final int accelCalibrateBtn = 2;
  final int autoBalanceBtn = 3;
  private double pitchBias = 0;
  private double gravity = -9.81;

  // drive modifiers mapping
  final int driveSpeedUpBtn = 5;
  final int driveSpeedDownBtn = 6;
  final int driveReverseBtn = 1;

  // functional button mapping
  final int funcReverseBtn = 1;
  final int openGripper = 5;
  final int closeGripper = 6;
  //final int func1Btn = 1;
  //final int func2Btn = 2;
  //final int func3Btn = 3;
  //final int func4Btn = 4;

  //drive motors and control objects
  private CANSparkMax moveMotorID5;
  private CANSparkMax moveMotorID7;
  private MotorControllerGroup rightMoveMotors;
  private CANSparkMax moveMotorID6;
  private CANSparkMax moveMotorID8;
  private MotorControllerGroup leftMoveMotors;
  private DifferentialDrive differentialDrive;
  private double driveSpeed = 1;

  //acceleration limiters
  private SlewRateLimiter limiter0;
  private SlewRateLimiter limiter1;
  private SlewRateLimiter limiter2;
  private SlewRateLimiter limiter3;

  // navX MXP using USB
  private AHRS gyro;
  private GenericEntry gyroCompassEntry;
  //april tag
  AprilTagTracker aprilTagTracker;
// for colour actions
  private boolean isRed = false;



  public Arm arm; 

    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { // init == initiate == happens once and at the beginning
    //colour selection and actions
  Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
      isRed = true;
    }
    if (ally.get() == Alliance.Blue) {
      isRed = false;
    }
}

    aprilTagTracker = new AprilTagTracker("Arducam_OV9281_USB_Camera");
    arm = new Arm();
    // Places a compass indicator for the gyro heading on the dashboard
    gyro = new AHRS(SerialPort.Port.kUSB);
    ShuffleboardTab compassTab = Shuffleboard.getTab("Compass");
    gyroCompassEntry = compassTab.add("Gyro Compass", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .getEntry();

    limiter0 = new SlewRateLimiter(2); //x-axis drive
    limiter1 = new SlewRateLimiter(1.5); //y-axis drive
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Initialize joystick object
    joystick = new Joystick(0); // Controller in port 0
    joystick2 = new Joystick(1); // Controller in port 1


    // Drive motor object initialization
    moveMotorID5 = new CANSparkMax(5, MotorType.kBrushless); // NEO motor with CAN ID 5, right side
    moveMotorID6 = new CANSparkMax(6, MotorType.kBrushless); // NEO motor with CAN ID 6, left side
    moveMotorID7 = new CANSparkMax(7, MotorType.kBrushless); // NEo motor with CAN ID 7, right side
    moveMotorID8 = new CANSparkMax(8, MotorType.kBrushless); // NEO motor with CAN ID 8, left side

    // Fix wiring inversion
    moveMotorID7.setInverted(true); // wiring thing, motor is flipped, bad wiring

    // Initialize motor groups
    moveMotorID7.follow(moveMotorID5); // because they are on the same side
    moveMotorID8.follow(moveMotorID6); // ^
  

    // Differential drive object initialization
    differentialDrive = new DifferentialDrive(moveMotorID6, moveMotorID5); //class that handles arcade drive, (one stick controller)
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    aprilTagTracker.UpdateTracker();
    //detects if there is a note in the intake
    arm.checkSensorandNotify();
    // Read the current yaw angle from the gyro
    double yawAngle = gyro.getYaw();
    // Convert the yaw angle to a 0-360 range for compass heading
    double compassHeading = yawAngle < 0 ? 360 + yawAngle : yawAngle;
    // Update the Shuffleboard compass widget with the current heading
    gyroCompassEntry.setDouble(compassHeading);
    //aprilTags and actions
    if ((isRed && aprilTagTracker.HasTargetWithId(4)) || (!isRed && aprilTagTracker.HasTargetWithId(7))) {
      SmartDashboard.putString("AprilTag", "Middle shooter AprilTag detected");
      double yaw;
      double pitch;
      double AprilTagHeight = 132;
      double cameraHeight = 0;
      double distance;
      if (isRed)  {
        yaw = aprilTagTracker.GetTargetWithId(4).yaw;
        pitch = aprilTagTracker.GetTargetWithId(4).pitch;

      }
      else {
        yaw = aprilTagTracker.GetTargetWithId(7).yaw;
        pitch = aprilTagTracker.GetTargetWithId(7).pitch;
      }
      distance = AprilTagHeight - cameraHeight / Math.tan(pitch);



     }
    else {
      SmartDashboard.putString("AprilTag", "No AprilTag detected");
    }
    if ((isRed && aprilTagTracker.HasTargetWithId(5)) || (!isRed && aprilTagTracker.HasTargetWithId(6))) {


    
    }else{
    
    }
    if ((isRed && aprilTagTracker.HasTargetWithId(11)) || (!isRed && aprilTagTracker.HasTargetWithId(16))) {

    
    }else{
  
    }
    if ((isRed && aprilTagTracker.HasTargetWithId(12)) || (!isRed && aprilTagTracker.HasTargetWithId(15))) {

    
    }else{
    }

    if ((isRed && aprilTagTracker.HasTargetWithId(13)) || (!isRed && aprilTagTracker.HasTargetWithId(14))) {

    
    }else{
    }
    }

   // periodic == happens like every milisecond

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //drives robot
    differentialDrive.arcadeDrive(limiter0.calculate(joystick.getX() * driveSpeed * 0.5), limiter1.calculate(joystick.getY() * driveSpeed));
    //}
    //eventually will define what each word means, e.g limiter1 refers safety in limiting acceleration speed
    if (Math.abs(joystick2.getY()) <= 0.1)
    {
   //  armMotor1.set(0);
   //   armMotor2.set(0); //outputs changed to 0, results in no motor function
    }
    else
    {
   //   armMotor1.set(-1*joystick2.getY());
   //   armMotor2.set(-1*joystick2.getY()); // output value == getY (joystick) and -1 because wiring
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
