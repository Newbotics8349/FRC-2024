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

import javax.lang.model.util.ElementScanner14;

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
  //private static final String blueMiddleAuto = "Blue Middle Auto";
  //private static final String redMiddleAuto = "Red Middle Auto";
  //private static final String redOneNoteAuto = "Red One Note Auto";
  //private static final String blueOneNoteAuto = "Blue One Note Auto";

  private static final String rightOneNote = "Right One Note Auto";
  private static final String leftOneNote = "Left One Note Auto";
  private static final String middleTwoNote = "Middle Two Note Auto";



  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // controls
  private Joystick joystick; //joystick used for DRIVING
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
  //final int funcReverseBtn = 1;
  //final int openGripper = 5;
  //final int closeGripper = 6;
  //final int buttonArmUp = 2;
  //final int buttonArmDown = 3;
  final int intakeBtn = 7;
  final int shootBtn = 8;
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
  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;

  //acceleration limiters
  private SlewRateLimiter limiter0;
  private SlewRateLimiter limiter1;
  private SlewRateLimiter limiter2;
  private SlewRateLimiter limiter3;

  // navX MXP using USB
  private AHRS gyro;
  private GenericEntry gyroCompassEntry;
  //april tag
// for colour actions
  public AprilTagTracker aprilTagTracker;
  private boolean isRed = false;
  public Arm arm; 
  public double yawAngle;
  public double minAngle;
  public double maxAngle;

  public double autoAngle;
  public int autoStepNum = 0;
  public double inchesPerEnocderClick = 1.76;
  public double distanceInInchesToMove;
  public double autoXValueTwoNote; // used to get angle for middle two note auto shooting angle
    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //determines shooting angle from distance obtained from Apriltag and shooting speed


  public double projectileAngle(double distance, double Vi) {

    for (double i = 40; i < 90; i+=0.1) {
      double y = distance * Math.tan(Math.toRadians(i)) + 4.905 * Math.pow(distance, 2)/Math.pow(Vi, 2)/Math.pow(Math.cos(Math.toRadians(i)), 2);
      if (y > 1.98) {
        minAngle = i;
        break;
      }
    }

    for (double i = 90; i >= 40; i-=0.1) {
      double y = distance * Math.tan(Math.toRadians(i)) + 4.905 * Math.pow(distance, 2)/Math.pow(Vi, 2)/Math.pow(Math.cos(Math.toRadians(i)), 2);
      if (y < 2.11) {
        maxAngle = i;
        break;
      }
    }
    autoAngle = (maxAngle + minAngle) / 2;

    return (minAngle + maxAngle)/2;      //Returns the average value, needs testing
  }

  @Override
  public void robotInit() { // init == initiate == happens once an/d at the beginning
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

    //ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    //autoSelection = autoTab.add("Auto Selection", 1);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("blueMiddleAuto", blueMiddleAuto); // Blue, Big Cheese
    //m_chooser.addOption("redMiddleAuto", redMiddleAuto); // Red, Big Cheese
    //m_chooser.addOption("redOneNoteAuto", redOneNoteAuto); // Red, small cheese
    //m_chooser.addOption("blueOneNoteAuto", blueOneNoteAuto); // Blue, small cheese
    m_chooser.addOption("leftOneNote", leftOneNote); // Both sides, if robot needs to turn left, small cheese
    m_chooser.addOption("rightOneNote", rightOneNote); // Both sides, if robot needs to turn left, small cheese
    m_chooser.addOption("middleTwoNote", middleTwoNote); // Both sides, no turning, big cheese
    m_chooser.addOption("kCustomAuto", kCustomAuto); // test/temporary
    SmartDashboard.putData("Auto choices", m_chooser);
    // Initialize joystick object
    joystick = new Joystick(0); // Controller in port 0
    joystick2 = new Joystick(1); // Controller in port 1


    // Drive motor object initialization
    moveMotorID5 = new CANSparkMax(5, MotorType.kBrushless); // NEO motor with CAN ID 5, right side
    moveMotorID6 = new CANSparkMax(6, MotorType.kBrushless); // NEO motor with CAN ID 6, left side
    moveMotorID7 = new CANSparkMax(7, MotorType.kBrushless); // NEo motor with CAN ID 7, right side
    moveMotorID8 = new CANSparkMax(8, MotorType.kBrushless); // NEO motor with CAN ID 8, left side
    // climber motor initialization
    climberMotor1 = new CANSparkMax(10, MotorType.kBrushed);
    climberMotor2 = new CANSparkMax(11, MotorType.kBrushed);

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
    yawAngle = gyro.getYaw();
    // Convert the yaw angle to a 0-360 range for compass heading
    double compassHeading = yawAngle < 0 ? 360 + yawAngle : yawAngle;
    // Update the Shuffleboard compass widget with the current heading
    gyroCompassEntry.setDouble(compassHeading);
    //aprilTags and actions
    //MAKE ONLY HAPPEN WHEN BUTTON PUSHED!!!
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
      arm.moveToPosition(projectileAngle(distance, 6.0)); //need initial velocity
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
  
   public void autoMove(double lW, double rW)
   {
     moveMotorID6.set(lW); //LEFT SIDE GOING FORWARD
     moveMotorID8.set(lW);
     moveMotorID5.set(rW); //RIGHT SIDE, GOING BACK
     moveMotorID7.set(rW);
   }

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

        case leftOneNote: //=====================================================================================================
            
          switch(autoStepNum){
            case 1: //move arm up
                arm.moveToPosition(15);
              if (arm.getArmAngle() == 15)
              {
                arm.setMotorPower(0);
                autoStepNum = 2;
                break;
              }
            case 2:
              arm.shooter(0.5);
              if (arm.checkSensorandNotify() == true)
              {
                arm.shooter(0);
                autoStepNum = 3;
                break;
              }
            case 3:
              distanceInInchesToMove = 12;
              if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove / inchesPerEnocderClick)
              {
                autoMove(0.5, 0.5);
              }
              else
              {
                autoMove(0,0);
                autoStepNum = 4;
                break;
              }
            case 4:
              //turning right
              if (gyro.getYaw() != 60)
              {
                autoMove(0.2,-0.2);
              }
              else 
              {
                autoMove(0,0);
                autoStepNum = 5;
                break;
              }
            case 5:
              distanceInInchesToMove = 65;
              if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove / inchesPerEnocderClick)
              {
                autoMove(0.5, 0.5);
              }
              else
              {
                autoMove(0,0);
                autoStepNum = 0;
                break;
              }
            default:

              break;
          }
          

        case rightOneNote: //=========================================================================================================

          switch(autoStepNum){
            case 1:
                arm.moveToPosition(15);
              //}
              if (arm.getArmAngle() == 15)
              {
                arm.setMotorPower(0);
                autoStepNum = 2;
                break;
              }
            case 2:
              arm.shooter(0.5);
              if (arm.checkSensorandNotify() == true)
              {
                arm.shooter(0);
                autoStepNum = 3;
                break;
              }
            case 3:
              distanceInInchesToMove = 12;
              if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove / inchesPerEnocderClick)
              {
                autoMove(0.5, 0.5);
              }
              else
              {
                autoMove(0,0);
                autoStepNum = 4;
                break;
              }
            case 4:
              //turning left
              if (gyro.getYaw() != -60)
              {
                autoMove(-0.2,0.2);
              }
              else 
              {
                autoMove(0,0);
                autoStepNum = 5;
                break;
              }
            case 5:
              distanceInInchesToMove = 65;
              if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove / inchesPerEnocderClick)
              {
                autoMove(0.5, 0.5);
              }
              else
              {
                autoMove(0,0);
                autoStepNum = 0;
                break;
              }
            default:

            break;
              
          }


        case middleTwoNote: //====================================================================================================

        switch(autoStepNum){
          case 1: //arm goes up
              arm.moveToPosition(15);
            if (arm.getArmAngle() == 15)
            {
              arm.setMotorPower(0);
              autoStepNum = 2;
              break;
            }
          case 2: // shoot
            arm.shooter(0.5);
            if (arm.checkSensorandNotify() == true)
            {
              arm.shooter(0);
              autoStepNum = 3;
              break;
            }

          case 3: // move arm down
            arm.moveToPosition(0);
            if (arm.getArmAngle() == 0)
            {
              arm.setMotorPower(0);
              autoStepNum = 4;
              break;
            }

          case 4: //move forward to other note
            //distanceInInchesToMove = 65;
            arm.intake(0.5);
            if (arm.checkSensorandNotify() == true)
            {
              autoMove(0.5, 0.5);
              autoXValueTwoNote = moveMotorID5.getEncoder().getPosition();
            }
            else
            {
              autoMove(0,0);
              arm.intake(0);
              autoStepNum = 5;
              break;
            }
          case 5:
            double autoShoot = Math.atan(80 / autoXValueTwoNote);
            arm.moveToPosition(autoShoot);
            if (arm.getArmAngle() == autoShoot)
            {
              arm.setMotorPower(0);
              autoStepNum = 6;
              break;
            }
          case 6:
            arm.shooter(0.5);
            if (arm.checkSensorandNotify() == true)
            {
              arm.shooter(0);
              autoStepNum = 0;
              break;
            }
            
          default:

          break;
        }

      

        

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
      double pos = arm.getArmAngle();
      //drives robot
      //if (enableDrive){
      differentialDrive.arcadeDrive(limiter0.calculate(joystick.getX() * driveSpeed * 0.5), limiter1.calculate(joystick.getY() * driveSpeed));
      //}
      //eventually will define what each word means, e.g limiter1 refers safety in limiting acceleration speed
      if (Math.abs(joystick2.getY()) <= 0.1)
      {
        arm.armMotor1.set(0);
        arm.armMotor2.set(0); //outputs changed to 0, results in no motor function
      }
      else
      {
        arm.armMotor1.set(-1*joystick2.getY());
        arm.armMotor2.set(-1*joystick2.getY()); // output value == getY (joystick) and -1 because wiring
      }
      //check if intake button pressed

      //if joystick up, pos += a;

      if (joystick2.getY() > 0 && pos < 90) // check to see if works
      {
        arm.moveToPosition(pos+10);
      }
      else if (joystick2.getY() < 0 && pos > 0)
      {
        arm.moveToPosition(pos-10);
      }

      if (joystick.getRawButton(7))
      {
        arm.intake(0.7); //get actual power variable
      }
      if (joystick.getRawButton(8))
      {
        arm.shooter(0.7); //^^^
      }
      //climber
      if(joystick.getRawButton(2)) 
      {
        climberMotor1.set(0.5);
        climberMotor2.set(0.5);
      }
      else
      {
        climberMotor1.set(0);
        climberMotor2.set(0);
      }
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
