/*     */ package frc.robot;
/*     */ 
/*     */ import edu.wpi.first.cameraserver.CameraServer;
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
/*     */   
/*     */   private boolean STATE_HAS_SHOT = false;
/*     */   
/*     */   private boolean STATE_IS_SHOOTING = false;
/*     */   private boolean STATE_IS_WINCHING = false;
  private Timer m_timer = new Timer();
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
/*     */ 
/*     */     
    CameraServer.startAutomaticCapture();
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

