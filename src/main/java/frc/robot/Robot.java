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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.AprilTagTracking.*;
import edu.wpi.first.wpilibj.Servo;


import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.util.Map;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import frc.robot.Arm;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Declaration of autonomous routine names
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    // private static final String blueMiddleAuto = "Blue Middle Auto";
    // private static final String redMiddleAuto = "Red Middle Auto";
    // private static final String redOneNoteAuto = "Red One Note Auto";
    // private static final String blueOneNoteAuto = "Blue One Note Auto";

    private static final String rightOneNote = "Right One Note Auto";
    private static final String leftOneNote = "Left One Note Auto";
    private static final String middleOneNote = "Middle Two Note Auto";
    private static final String straightUpAuto = "Straight Up Auto";
    private static final String doubleDouble = "Double Double";


    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // controls
    private Joystick joystick; // joystick used for DRIVING

    // functional button mapping
    // final int funcReverseBtn = 1;
    // final int openGripper = 5;
    // final int closeGripper = 6;
    // final int buttonArmUp = 2;
    // final int buttonArmDown = 3;
    final int intakeBtn = 7;
    final int shootBtn = 8;
    // final int func1Btn = 1;
    // final int func2Btn = 2;
    // final int func3Btn = 3;
    // final int func4Btn = 4;

    // drive motors and control objects
    private CANSparkMax moveMotorID5;
    private CANSparkMax moveMotorID7;
    private CANSparkMax moveMotorID6;
    private CANSparkMax moveMotorID8;
    private DifferentialDrive differentialDrive;
    private double driveSpeed = 1;
    private Spark climberMotor1;
    private Spark climberMotor2;
    public Servo LeftServo;
    public Servo RightServo;


    // acceleration limiters
    private SlewRateLimiter limiter0; // Used for limiting forward speed in arcade drive call
    private SlewRateLimiter limiter1; // Used for limiting turning speed in arcade drive call

    // navX MXP using USB
    private AHRS gyro;
    private GenericEntry gyroCompassEntry;
    // AprilTag detection system
    public AprilTagTracker aprilTagTracker;
    //during games, gets error messages from referencing camera, no overall impact, just errors
    // for colour actions
    private boolean isRed = false;
    // ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,m subsystem
    public Arm arm;
    final int verticalReading = 461;
    final double potValueToDegrees = 180.0/3270.0;

    public double autoAngle;
    public int autoStepNum = 1;
    public final double inchesPerEnocderClick = 1.76; // TODO: Update this value based on the 42 encoder ticks/2PI radians and the gear ratio on the drivetrain
    public double distanceInInchesToMove;
    public double autoXValueTwoNote; // used to get angle for middle two note auto shooting angle
    //public int temp = 0;
    public int dPadValue;
    public Timer timer;
    public Timer autoTimer;
    public boolean intakeLock = false;
    public double angleRef;
    public boolean testy = true;

    /**
     * Determines the angle needed to shoot the ring at.
     * Calculates the angle specifically to reach a height of 1.98-2.11 metres (the
     * height of the speaker hole).
     * 
     * @param distance, the horizontal distance to the speaker, obtained from vision
     *                  system
     * @param Vi,       the speed of the shooter
     * @return the angle in degrees, measured from the horizontal, to shoot at
     */
    public double projectileAngle(double distanceInM, double ViInMps) { // TODO: Update arm.getArmDistanceFromCamera and arm.getArmHeight
        double maxAngle = -1;
        double minAngle = -1;
        for (double i = 40; i < 90; i += 0.1) {
            double shooterHeight = arm.getArmHeight(i);
            double shooterExtraDistance = arm.getArmDistanceFromCamera(i);
            double y = (distanceInM+shooterExtraDistance) * Math.tan(Math.toRadians(i))
                    + 4.905 * Math.pow(distanceInM+shooterExtraDistance, 2) / Math.pow(ViInMps, 2)
                            / Math.pow(Math.cos(Math.toRadians(i)), 2);
            if (y > 1.98-shooterHeight && y < 2.11-shooterHeight) {
                if (minAngle == -1) minAngle = i;
                if (maxAngle == -1) maxAngle = i;

                if (i>maxAngle) maxAngle = i;
                if (i>maxAngle) minAngle = i;

                minAngle = i;
                break;
            }
        }

        if (minAngle != -1 && maxAngle != -1)
            return (minAngle + maxAngle) / 2; // Returns the average value, needs testing
        return -1;  // Default result returned when no valid angles were found
    }

    /**
     * Moves the robot in autonomous mode.
     * 
     * @param lW, the power to set the motors on the left of the drive train
     * @param rW, the power to set the motors on the right of the drive train
     */
    public void autoMove(double lW, double rW) {
        moveMotorID6.set(lW); // LEFT SIDE GOING FORWARD
        moveMotorID8.set(lW);
        moveMotorID5.set(rW); // RIGHT SIDE, GOING BACK
        moveMotorID7.set(rW);
    }

    /**
     * Automatically aim and shoot at the speaker when the center AprilTag is detected.
     * 
     */
    public void automaticallyShoot()
    {
        // Check that AprilTag is detected
        if ((isRed && aprilTagTracker.HasTargetWithId(4)) || (!isRed && aprilTagTracker.HasTargetWithId(7))) {
            // Determine AprilTag location relative to camera
            double yaw;
            double pitch;
            double AprilTagHeight = 132; // Constant height from manual, unit is cm
            double cameraHeight = 0; // TODO: Get value from robot, height from ground to centre of camera   
            double distance;
            if (isRed) {
                yaw = aprilTagTracker.GetTargetWithId(4).GetYaw();
                pitch = aprilTagTracker.GetTargetWithId(4).GetPitch();

            } else {
                yaw = aprilTagTracker.GetTargetWithId(7).GetYaw();
                pitch = aprilTagTracker.GetTargetWithId(7).GetPitch();
            }

            // Translating yaw
            // Done simply by trying to get yaw from AprilTag close to 0, that means we are directly aligned with the AprilTag
            final double turningKp = 0.2; // TODO: Adjust as needed, find max value that does not cause the robot to move back and forth around yaw=0
            double turningPower = yaw*turningKp;
            autoMove(turningPower, -turningPower);

            // Translating pitch
            // Run the shooter motors so that we can determine speed
            arm.prepareToShoot(1); // TODO: Adjust as needed, power of shooting motors

            // Calculate the angle needed to fire at the speaker
            distance = (AprilTagHeight - cameraHeight / Math.tan(pitch)); // Distance from camera to Speaker wall

            double targetAngle = projectileAngle(distance/100, arm.calculateShooterSpeed()/100);

            // If acceleration is low and the arm is at the needed angle, fire
            final double accelerationThreshold = 0.1; // TODO: Adjust as needed, determines when the speed of the shooter motors has leveled out by considering the average velocity between two speed measurements
            final double armThreshold = 2; // TODO: Adjust as needed, determines the allowable error for the arm shooter direction, we can fire when the arm angle is +-armThreshold the target angle
            final double yawThreshold = 2; // TODO: Adjust as needed, determines the allowable error in yaw

            // If motor speed is level, and aimed position is within allowable threshold, fire
            if (arm.getAcceleration() < accelerationThreshold && Math.abs(arm.getArmAngle()-targetAngle)<armThreshold && Math.abs(yaw)<yawThreshold)
                arm.sendNoteToShooter(1); // TODO: Change as needed if 100% is too violent
            else
                arm.sendNoteToShooter(0);
        }
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() { // init == initiate == happens once an/d at the beginning
        // Determine robot colour
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isRed = true;
            }
            if (ally.get() == Alliance.Blue) {
                isRed = false;
            }
        }

        // Initialize arm and vision system components
        
        aprilTagTracker = new AprilTagTracker("Arducam_OV9281_USB_Camera");
        arm = new Arm();

        timer = new Timer();

        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 640, 420, 30);
        //CameraServer.getVideo();
        //CameraServer.putVideo("Processed", 640, 420);

        // Places a compass indicator for the gyro heading on the dashboard
        gyro = new AHRS(SerialPort.Port.kUSB);
        ShuffleboardTab compassTab = Shuffleboard.getTab("Compass");
        gyroCompassEntry = compassTab.add("Gyro Compass", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .getEntry();

        // TODO: Adjust limiters as needed (smooths acceleration when driving the robot)
        limiter0 = new SlewRateLimiter(2); // x-axis drive
        limiter1 = new SlewRateLimiter(1.5); // y-axis drive

        // ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
        // autoSelection = autoTab.add("Auto Selection", 1);

        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("straightUpAuto", straightUpAuto); // Just getting over the line, the smallest cheese

        m_chooser.addOption("leftOneNote", leftOneNote); // Both sides, if robot needs to turn left, small cheese
        m_chooser.addOption("rightOneNote", rightOneNote); // Both sides, if robot needs to turn left, small cheese
        m_chooser.addOption("middleOneNote", middleOneNote); // Both sides, no turning, big cheese
        m_chooser.addOption("doubleDouble", doubleDouble); // Both sides, no turning, big cheese
        m_chooser.addOption("kCustomAuto", kCustomAuto); // test/temporary
        SmartDashboard.putData("Auto choices", m_chooser);

        // Initialize joystick object
        joystick = new Joystick(0); // Controller in port 0

        // Drive motor object initialization
        moveMotorID5 = new CANSparkMax(5, MotorType.kBrushless); // NEO motor with CAN ID 5, right side
        moveMotorID6 = new CANSparkMax(6, MotorType.kBrushless); // NEO motor with CAN ID 6, left side
        moveMotorID7 = new CANSparkMax(7, MotorType.kBrushless); // NEo motor with CAN ID 7, right side
        moveMotorID8 = new CANSparkMax(8, MotorType.kBrushless); // NEO motor with CAN ID 8, left side
        // climber motor initialization
        climberMotor1 = new Spark(1); //PMW port 1, right side
        climberMotor2 = new Spark(2); //PMW port 2, left side
        LeftServo = new Servo(3); //negative
        RightServo = new Servo(4);

        //LeftServo.set(0);
        //RightServo.set(0);

        // Fix wiring inversion
        // TODO: Invert motors running backwards to what is expected during testing
        // moveMotorIDX.setInverted(true); // wiring thing, motor is flipped, bad wiring

        // Initialize motor groups in drivetrain
        moveMotorID7.follow(moveMotorID5); // because they are on the same side of the drive train
        moveMotorID8.follow(moveMotorID6); // ^

        // Differential drive object initialization
        differentialDrive = new DifferentialDrive(moveMotorID6, moveMotorID5); // class that handles arcade drive, (one
                                                                               // stick controller)
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        //arm.getArmAngle();
        SmartDashboard.putString("Left Servo Position", String.valueOf(LeftServo.getAngle())); 
        SmartDashboard.putString("Right Servo Position", String.valueOf(RightServo.getAngle())); 
        //double test = arm.getArmAngle(); // test == what I want the angle to be, need a variable 

        arm.absAngle();
        SmartDashboard.putString("note timer", String.valueOf(timer.get())); 


        // Updates the target's seen by the vision system
        aprilTagTracker.UpdateTracker();

        // Tells the driver if the speaker's center April tag is located
        if ((isRed && aprilTagTracker.HasTargetWithId(4)) || (!isRed && aprilTagTracker.HasTargetWithId(7)))
            SmartDashboard.putString("AprilTag", "Middle Speaker AprilTag detected");
        else
            SmartDashboard.putString("AprilTag", "No Speaker AprilTag detected");

        // Provides diagnostic data to the driver
        // Detects if there is a note in the intake
        arm.hasNoNote();

        // Displays robot compass heading to the driver
        // Read the current yaw angle from the gyro
        double yawAngle = gyro.getYaw();
        // Convert the yaw angle to a 0-360 range for compass heading
        double compassHeading = yawAngle < 0 ? 360 + yawAngle : yawAngle;
        // Update the Shuffleboard compass widget with the current heading
        gyroCompassEntry.setDouble(compassHeading);

        // aprilTags and actions
        // MAKE ONLY HAPPEN WHEN BUTTON PUSHED!!!
        // if ((isRed && aprilTagTracker.HasTargetWithId(4)) || (!isRed &&
        // aprilTagTracker.HasTargetWithId(7))) {
        // SmartDashboard.putString("AprilTag", "Middle shooter AprilTag detected");
        // double yaw;
        // double pitch;
        // double AprilTagHeight = 132;
        // double cameraHeight = 0;
        // double distance;
        // if (isRed) {
        // yaw = aprilTagTracker.GetTargetWithId(4).GetYaw();
        // pitch = aprilTagTracker.GetTargetWithId(4).GetPitch();

        // } else {
        // yaw = aprilTagTracker.GetTargetWithId(7).GetYaw();
        // pitch = aprilTagTracker.GetTargetWithId(7).GetPitch();
        // }
        // distance = AprilTagHeight - cameraHeight / Math.tan(pitch);
        // arm.prepareToShoot(0.75);

        // arm.moveToPosition(projectileAngle(distance, arm.calculateShooterSpeed()));
        // // need initial velocity
        // } else {
        // SmartDashboard.putString("AprilTag", "No AprilTag detected");
        // }
    }

    // periodic == happens like every milisecond
    @Override
    public void autonomousInit() {
        autoTimer = new Timer();
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
        moveMotorID5.getEncoder().setPosition(0);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("auto timer", String.valueOf(autoTimer.get())); 
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;

            case straightUpAuto: // JUST MOVE FORWARD A BIT
                autoTimer.start();
                SmartDashboard.putString("auto timer", String.valueOf(autoTimer.get())); 
                            autoMove(0.2, -0.2);
                            if (autoTimer.get() >= 3) {
                                autoMove(0, 0);
                            break;
                            }
            case leftOneNote: // =====================================================================================================
                              //EVENTUALLY, MAKE ANGLE TWO NOTE SYSTEM, WHAT IS HERE NOW, BUT WORKS


                        //else {
                            //autoMove(0, 0);
                            //autoStepNum = 4;
                            //break;
                        //}
                    /* 
                    case 1: // move arm up
                        arm.moveToPosition(-16);
                        if (Math.abs(arm.getArmAngle()-55) < 3) { //change arm angle to encoder
                            arm.setMotorPower(0);
                            autoStepNum = 2;
                            break;
                        }
                    case 2:
                        arm.shoot(0.5);
                        if (arm.hasNoNote()) 
                        {
                            arm.shoot(0);
                            autoStepNum = 3;
                            break;
                        }
                    case 3:
                        distanceInInchesToMove = 12;
                        if (Math.abs(moveMotorID5.getEncoder().getPosition())*inchesPerEnocderClick < distanceInInchesToMove) {
                            autoMove(0.5, 0.5);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 4;
                            break;
                        }
                    case 4:
                        // turning right
                        if (gyro.getYaw() != 60) {
                            autoMove(0.2, -0.2);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 5;
                            break;
                        }
                    case 5:
                        distanceInInchesToMove = 65;
                        if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove
                                / inchesPerEnocderClick) {
                            autoMove(0.5, 0.5);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 0;
                            break;
                        }
                    */
                //    default:

                //        break;
                //}

            case rightOneNote: // =========================================================================================================
                               //EVENTUALLY, MAKE ANGLE TWO NOTE SYSTEM, WHAT IS HERE NOW, BUT WORKS
                switch (autoStepNum) {
                    case 1:
                        arm.PID(55);
                        if (Math.abs(arm.getArmAngle()-55) < 3) {
                            arm.setMotorPower(0);
                            autoStepNum = 2;
                            break;
                        }
                        
                    case 2:
                        arm.shoot(0.5);
                        if (arm.hasNoNote()) 
                        {
                            arm.shoot(0);
                            autoStepNum = 3;
                            break;
                        }
                    case 3:
                        distanceInInchesToMove = 12;
                        if (Math.abs(moveMotorID5.getEncoder().getPosition())*inchesPerEnocderClick < distanceInInchesToMove) {
                            autoMove(0.5, 0.5);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 4;
                            break;
                        }
                    case 4:
                        // turning left
                        if (gyro.getYaw() != -60) {
                            autoMove(-0.2, 0.2);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 5;
                            break;
                        }
                    case 5:
                        distanceInInchesToMove = 65;
                        if (Math.abs(moveMotorID5.getEncoder().getPosition()) < distanceInInchesToMove
                                / inchesPerEnocderClick) {
                            autoMove(0.5, 0.5);
                        } else {
                            autoMove(0, 0);
                            autoStepNum = 0;
                            break;
                        }
                    default:

                        break;

                }

            case doubleDouble: //eventually, find a way to pick up another note
                // ====================================================================================================
                    autoTimer.start();
                    SmartDashboard.putString("false == no longer shoot", String.valueOf(testy)); 
                    if(testy)
                    {
                        arm.autoAngle(300);
                    }
                    SmartDashboard.putString("auto timer", String.valueOf(autoTimer.get())); 
                    if (autoTimer.get() >= 3) {
                        SmartDashboard.putString("OI!", "OI!!!!!!");
                        arm.shoot(0.7);
                        arm.intake(0.7);
                        if (autoTimer.get() >= 5)
                        {
                            arm.shoot(0);
                        
                        
                        
                        if (autoTimer.get() >= 6) //HERE
                        {
                            testy = false;
                            SmartDashboard.putString("auto timer", String.valueOf(testy)); 
                            arm.autoAngle(316);
                            autoMove(0.2, -0.2);
                            //arm.shoot(0);
                            arm.intake(0.7); //MAKE SURE THE SENSOR DOESN'T BREAK THE NOTE DURING TIME
                            
                            if (autoTimer.get() >= 9) { // EVENTUALLY CHANGE 9, INSTEAD TO A DISTANCE THAT PICKS UP NOTE //HERE
                                autoMove(0, 0);
                                arm.intake(0);
                                arm.autoAngle(290); // CHECK ANGLE, MAKE WORK //HERE
                                if (autoTimer.get() >= 10) //HERE
                                {
                                    arm.shoot(0.7);
                                    if (autoTimer.get() >= 13) //HERE
                                    {
                                        arm.shoot(0);
                                        break;
                                    }
                                }
                                
                            }
                        }
                        }
                        
                    }

            case middleOneNote: //eventually, find a way to pick up another note
            // ====================================================================================================
                autoTimer.start();
                arm.autoAngle(300);
                
                SmartDashboard.putString("auto timer", String.valueOf(autoTimer.get())); 
                if (autoTimer.get() >= 2) {
                    arm.shoot(0.7);
                    if (autoTimer.get() >= 4.5)
                    {
                        autoMove(0.2, -0.2);
                        arm.shoot(0);
                        if (autoTimer.get() >= 7) {
                            autoMove(0, 0);
                            break;
                        }
                    }
                }
                /*
                switch (autoStepNum) {
                    case 1: // arm goes up
                        arm.moveToPosition(55);
                        if (Math.abs(arm.getArmAngle()-55) < 3) {
                            arm.setMotorPower(0);
                            autoStepNum = 2;
                            break;
                        }
                    case 2: // shoot
                        arm.shoot(0.5);
                        if (arm.hasNoNote()) 
                        {
                            arm.shoot(0);
                            autoStepNum = 3;
                            break;
                        }

                    case 3: // move arm down
                        arm.moveToPosition(0);
                        if (Math.abs(arm.getArmAngle()) < 3) {
                            arm.setMotorPower(0);
                            autoStepNum = 4;
                            break;
                        }

                    case 4: // move forward to other note
                        // distanceInInchesToMove = 65
                        arm.intake(0.5);
                        if (arm.hasNoNote() == true) {
                            autoMove(0.5, 0.5);
                            autoXValueTwoNote = moveMotorID5.getEncoder().getPosition();
                        } else {
                            autoMove(0, 0);
                            arm.intake(0);
                            autoStepNum = 5;
                            break;
                        }
                    case 5:
                        double autoShoot = Math.atan(80 / autoXValueTwoNote);
                        arm.moveToPosition(autoShoot);
                        if (arm.getArmAngle() == autoShoot) {
                            arm.setMotorPower(0);
                            autoStepNum = 6;
                            break;
                        }
                    case 6:
                        arm.shoot(0.5);
                        if (arm.hasNoNote() == true) {
                            arm.shoot(0);
                            autoStepNum = 0;
                            break;
                        }

                    default:

                        break;
                }
                */

            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //arm.SetArmTo0();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() { // CONSIDER ADDING NEGATIVE TO DRIVESPEED, ORANGE BECOMES FRONT, GREEN BECOMES BACK

        // Drives robot using controller X and Y axis
        differentialDrive.arcadeDrive(limiter0.calculate(joystick.getX() * driveSpeed * 0.4),
                limiter1.calculate(joystick.getY() * -driveSpeed));
        // eventually will define what each word means, e.g limiter1 refers safety in
        // limiting acceleration speed

        // Pitching the arm


        /*
        dPadValue = joystick.getPOV();

        if (dPadValue == 0)
        {
            arm.setMotorPower(-0.5);
        }
        else if (dPadValue == 180)
        {
            arm.setMotorPower(0.5);
        }
        */        
        

        if (Math.abs(joystick.getThrottle()) > 0.1)
        {
            arm.setMotorPower(joystick.getThrottle());
            final double angleRef = arm.encoder.get() * 360;
            SmartDashboard.putString("should not change", String.valueOf(angleRef)); 


        }

        else if (joystick.getRawButton(2)) // SPEAKER AUTO ANGLE
            arm.autoAngle(300); //EVENTUALLY CHANGE SO THAT ANGLES ARE IN QUADRANT 3!
        
        else if (joystick.getRawButton(3)) //AMP AUTO ANGLE
            arm.autoAngle(208); //EVENTUALLY CHANGE SO THAT ANGLES ARE IN QUADRANT 3!
        
        else 
        {
            arm.setMotorPower(0);
            //arm.PID(angleRef);
        }

        //if (joystick.getRawButtonPressed(9))
        //    arm.SetArmTo0();
        



        /*
        if (Math.abs(joystick.getZ()) <= 0.1) { //TODO: Adjust deadzone as needed
            arm.setMotorPower(0);
        } else {
            arm.setMotorPower(joystick.getZ()); // Set power based on y axis of rightmost joystick
        }
        */

        // if joystick up, pos += a;
        // if (joystick.getY() > 0 && pos < 90) // check to see if works
        // {
        // arm.moveToPosition(pos + 10);
        // } else if (joystick.getY() < 0 && pos > 0) {
        // arm.moveToPosition(pos - 10);
        // }
        

        // Running intake
        if (joystick.getRawButton(6))
        {
            SmartDashboard.putString("false = intake?", String.valueOf(intakeLock)); 
            
            if (arm.hasNoNote() == false/* && temp == 0*/)  {// note detected //FIX THE THINGS THAT ARE TIME RELATED, MAKE SENSOR WORK (might have to use time anyway)
                timer.start(); //eventually remove time, make sensor work
                intakeLock = true;
                //arm.sendNoteToShooter(-0.2);
                //arm.sendNoteToShooter(0);
                SmartDashboard.putString("note timer", String.valueOf(timer.get())); 
                arm.sendNoteToShooter(0.2);

                if (timer.get() >= 3)
                {
                    arm.sendNoteToShooter(0);
                //    arm.intake(0);
                    timer.stop();
                    //timer.reset();
                }
            }
            else if (intakeLock == false)/*if(temp == 0)*/ {
                arm.intake(0.4); // get actual power variable
                SmartDashboard.putString("Sensor", "NO");
                timer.stop();
                timer.reset();
            }

        }

        

        //} else
        //    arm.intake(0);

        // Shooting
        else if (joystick.getRawButton(8)) // speaker shot
        { 
            arm.shoot(0.7); // ^^^
            intakeLock = false;
        }

        else if (joystick.getRawButton(7)) // dunk, amp shot
        {
            arm.shoot(0.2);
            intakeLock = false;
        }

        else
            arm.shoot(0); //THIS HAPPENS TWICE
            //temp = 0;

        //arm.giveAngle();

        if (joystick.getRawButton(5))
        {
            //arm.SetArmTo0();
        }
        /*

        if (joystick.getRawButton(9)) //unlock, 0 deg //when being pulled, off / go up // when not pull, down, locked
        {
            LeftServo.setAngle(0);
            RightServo.setAngle(20);
        }
        if (joystick.getRawButton(10)) //lock, 30 deg
        {
            LeftServo.setAngle(20);
            RightServo.setAngle(0);
        }
        */

        // Automatically Shooting
        //if (joystick.getRawButton(10)) // TODO: Remap button, I chose this randomly
        //    automaticallyShoot();
         // Only runs when AprilTag is detected

        // Set Arm angle to 0
        //if (joystick.getRawButton(9))
           //arm.encoder.reset();
            //arm.SetArmTo0();

        // Running climber
        if (joystick.getRawButton(1)) { // down// x
            LeftServo.setAngle(20);
            RightServo.setAngle(-20);
            SmartDashboard.putString("climb test", "1");
            climberMotor1.set(0.5); // TODO: Adjust climber speed as needed
            climberMotor2.set(0.5);
        } else if (joystick.getRawButton(4)) { // up // y
            LeftServo.setAngle(-20);
            RightServo.setAngle(20);
            SmartDashboard.putString("climb test", "4");
            climberMotor1.set(-0.5);
            climberMotor2.set(-0.5);
        } 
        else {
            SmartDashboard.putString("climb test", "7");
            climberMotor1.set(0);
            climberMotor2.set(0);
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
