/*     */ package frc.robot;
/*     */ 
/*     */ import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
/*     */ import edu.wpi.first.util.sendable.Sendable;
/*     */ import edu.wpi.first.wpilibj.PneumaticHub;
/*     */ import edu.wpi.first.wpilibj.PneumaticsModuleType;
/*     */ import edu.wpi.first.wpilibj.Solenoid;
/*     */ import edu.wpi.first.wpilibj.TimedRobot;
/*     */ import edu.wpi.first.wpilibj.Timer;
/*     */ import edu.wpi.first.wpilibj.XboxController;
/*     */ import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
/*     */ import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ public class Robot
/*     */   extends TimedRobot
/*     */ {
  private Drivetrain m_drivetrain = new Drivetrain();
  private Vision m_vision = new Vision(this.m_drivetrain);
  private Winch m_winch = new Winch(this.m_vision);
/*     */   
  private SendableChooser<Constants.AutoRoutines> m_chooser = new SendableChooser();
  private SendableChooser<Constants.TestAutos> m_testAutoChooser = new SendableChooser();
/*     */   
/*     */   private boolean STATE_HAS_SHOT = false;
/*     */   
/*     */   private boolean STATE_IS_SHOOTING = false;
/*     */   private boolean STATE_IS_WINCHING = false;
  private Timer m_timer = new Timer();
  private Timer m_testAutoTimer = new Timer();
/*     */   
  PneumaticHub m_ph = new PneumaticHub(1);
/*     */   
  Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private XboxController m_operator = new XboxController(1);
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void robotInit() {
    this.m_chooser.setDefaultOption("Go Ramp Shoot", Constants.AutoRoutines.kGoStraightRamp);
    this.m_chooser.addOption("Go Straight Shoot", Constants.AutoRoutines.kGoStraightLong);
    this.m_chooser.addOption("Go Straight No Shoot", Constants.AutoRoutines.kStraightLongNoShoot);
    this.m_chooser.addOption("Go Ramp No Shoot", Constants.AutoRoutines.kStraightRampNoShoot);
    this.m_chooser.addOption("Do Nothing", Constants.AutoRoutines.kDoNothing);
/*     */     
    SmartDashboard.putData((Sendable)this.m_chooser);
    this.m_testAutoChooser.setDefaultOption("Do Nothing", Constants.TestAutos.kDoNothing);
    this.m_testAutoChooser.addOption("Forward 5 Seconds", Constants.TestAutos.kForwardFiveSeconds);
    this.m_testAutoChooser.addOption("Backward 3 Seconds", Constants.TestAutos.kBackwardThreeSeconds);
    this.m_testAutoChooser.addOption("Turn Left", Constants.TestAutos.kTurnLeft);
    this.m_testAutoChooser.addOption("Turn Right", Constants.TestAutos.kTurnRight);
    this.m_testAutoChooser.addOption("Forward Then Circle", Constants.TestAutos.kForwardThenCircle);
    this.m_testAutoChooser.addOption("Square", Constants.TestAutos.kSquare);
    SmartDashboard.putData("Test Auto", this.m_testAutoChooser);
/*     */ 
/*     */     
    UsbCamera povCamera = CameraServer.startAutomaticCapture();
    povCamera.setResolution(320, 240);
    povCamera.setFPS(15);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   public void teleopPeriodic() {
    this.m_winch.operatorControl();
/*     */ 
/*     */     
    if (this.m_operator.getLeftBumper()) {
      this.m_vision.aim_GetinRange();
/*     */     } else {
      this.m_drivetrain.XboxDrive();
/*     */     } 
/*     */ 
/*     */ 
/*     */ 
/*     */     
    this.m_ph.enableCompressorAnalog(100.0D, 120.0D);
/*     */   }
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void disabledInit() {}
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void autonomousInit() {
    this.m_testAutoTimer.reset();
    this.m_testAutoTimer.start();
    this.m_drivetrain.resetEncoders();
    this.STATE_HAS_SHOT = false;
    this.STATE_IS_SHOOTING = false;
    this.STATE_IS_WINCHING = false;
/*     */   }
  static double setDriveDistance = 0.0D;
/*     */   
/*     */   static boolean setOverRampState = false;
/*     */   
/*     */   public void autonomousPeriodic() {
    Constants.TestAutos selectedTestAuto = this.m_testAutoChooser.getSelected();
    if (selectedTestAuto == Constants.TestAutos.kForwardFiveSeconds) {
      this.m_drivetrain.arcadeDrive(this.m_testAutoTimer.get() < 5.0 ? 0.35 : 0.0, 0.0);
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kBackwardThreeSeconds) {
      this.m_drivetrain.arcadeDrive(this.m_testAutoTimer.get() < 3.0 ? -0.35 : 0.0, 0.0);
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kTurnLeft) {
      this.m_drivetrain.arcadeDrive(this.m_testAutoTimer.get() < 2.0 ? 0.0 : 0.0, this.m_testAutoTimer.get() < 2.0 ? -0.35 : 0.0);
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kTurnRight) {
      this.m_drivetrain.arcadeDrive(0.0, this.m_testAutoTimer.get() < 2.0 ? 0.35 : 0.0);
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kForwardThenCircle) {
      if (this.m_testAutoTimer.get() < 5.0) {
        this.m_drivetrain.arcadeDrive(0.35, 0.0);
      } else if (this.m_testAutoTimer.get() < 8.0) {
        this.m_drivetrain.arcadeDrive(0.35, 0.25);
      } else {
        this.m_drivetrain.arcadeDrive(0.0, 0.0);
      }
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kSquare) {
      double time = this.m_testAutoTimer.get();
      if (time < 2.0 || (time >= 4.0 && time < 6.0) || (time >= 8.0 && time < 10.0) || (time >= 12.0 && time < 14.0)) {
        this.m_drivetrain.arcadeDrive(0.35, 0.0);
      } else if (time < 4.0 || (time >= 6.0 && time < 8.0) || (time >= 10.0 && time < 12.0)) {
        this.m_drivetrain.arcadeDrive(0.0, 0.35);
      } else {
        this.m_drivetrain.arcadeDrive(0.0, 0.0);
      }
      return;
    }
    if (selectedTestAuto == Constants.TestAutos.kDoNothing) {
      this.m_drivetrain.arcadeDrive(0.0, 0.0);
      return;
    }
    if (this.m_chooser.getSelected() == Constants.AutoRoutines.kGoStraightRamp || this.m_chooser.getSelected() == Constants.AutoRoutines.kStraightRampNoShoot) {
      setDriveDistance = -2.2D;
      setOverRampState = true;
/*     */     }
    else if (this.m_chooser.getSelected() == Constants.AutoRoutines.kGoStraightLong || this.m_chooser.getSelected() == Constants.AutoRoutines.kStraightLongNoShoot) {
/*     */       
      setDriveDistance = -5.2D;
/*     */     } else {
/*     */       
      setDriveDistance = 0.0D;
/*     */     } 
/*     */     
    if (this.m_chooser.getSelected() == Constants.AutoRoutines.kGoStraightRamp || this.m_chooser.getSelected() == Constants.AutoRoutines.kGoStraightLong) {
      if (!this.STATE_HAS_SHOT) {
        if (!this.m_winch.isWinched() || !this.m_timer.hasElapsed(1.0D)) {
          this.m_winch.winchToShoot();
          if (!this.STATE_IS_WINCHING) {
            this.m_timer.reset();
            this.m_timer.start();
            this.STATE_IS_WINCHING = true;
/*     */           } 
/*     */         } else {
          if (!this.STATE_IS_SHOOTING && !this.STATE_HAS_SHOT) {
            this.STATE_IS_SHOOTING = true;
            this.m_timer.reset();
            this.m_timer.start();
/*     */           } 
/*     */           
          if (this.STATE_IS_SHOOTING && !this.m_timer.hasElapsed(1.0D)) {
            this.m_winch.shoot(true);
/*     */           } else {
            this.m_winch.shoot(false);
            this.STATE_HAS_SHOT = true;
/*     */           } 
/*     */         } 
/*     */       }
/*     */       
      if (this.STATE_HAS_SHOT) {
        System.out.println("I'm here!!!!");
        this.m_drivetrain.driveStraightAndBalance();
/*     */       }
/*     */     
/*     */     } else {
/*     */       
      this.m_drivetrain.driveStraightAndBalance();
/*     */     } 
/*     */   }
/*     */ 
/*     */   
/*     */   public void robotPeriodic() {
    this.m_drivetrain.pushInfoToDash();
    this.m_vision.publishAlignedStatusToDash();
/*     */   }
/*     */ }

