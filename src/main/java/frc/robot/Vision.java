/*     */ package frc.robot;
/*     */ 
/*     */ import com.ctre.phoenix.motorcontrol.ControlMode;
/*     */ import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/*     */ import edu.wpi.first.math.controller.PIDController;
/*     */ import edu.wpi.first.math.util.Units;
/*     */ import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*     */ import org.photonvision.PhotonCamera;
/*     */ import org.photonvision.PhotonUtils;
/*     */ import org.photonvision.targeting.PhotonPipelineResult;
/*     */ import org.photonvision.targeting.PhotonTrackedTarget;
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ public class Vision
/*     */ {
  private TalonSRX PinkLight = new TalonSRX(0);
  private TalonSRX YellowLight = new TalonSRX(1);
/*     */   
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(5.0D);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30.0D);
  PhotonCamera camera = new PhotonCamera("OV5647");
/*     */ 
/*     */ 
/*     */   
  private double GOAL_RANGE_METERS = 0.84D;
  private final double TARGET_HEIGHT_METERS = Units.feetToMeters(2.0D);
  private int targetIndex = 0;
  private double GOAL_RANGE = 0.04D;
  private double GOAL_ALIGNMENT = 3.0D;
/*     */   
/*     */   private static boolean in_position = false;
/*     */   
/*     */   private Drivetrain m_drivetrain;
/*     */   
  final double LINEAR_P = 1.5D;
  final double LINEAR_D = 0.0D;
  PIDController forwardController = new PIDController(1.5D, 0.0D, 0.0D);
/*     */   
  final double ANGULAR_P = 0.02D;
  final double ANGULAR_D = 0.0D;
  PIDController turnController = new PIDController(0.02D, 0.0D, 0.0D);
/*     */   
/*     */   public Vision(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
/*     */   }
/*     */   
/*     */   public void publishAlignedStatusToDash() {
    PhotonPipelineResult result = this.camera.getLatestResult();
    boolean in_range = false;
    boolean is_aligned = false;
/*     */     
    if (result.hasTargets()) {
/*     */       
      double range = PhotonUtils.calculateDistanceToTargetMeters(this.CAMERA_HEIGHT_METERS, this.TARGET_HEIGHT_METERS, this.CAMERA_PITCH_RADIANS, 
/*     */ 
/*     */ 
/*     */           
          Units.degreesToRadians(((PhotonTrackedTarget)result.getTargets().get(this.targetIndex)).getPitch()));
      SmartDashboard.putNumber("Distance to Target", range);
/*     */ 
/*     */       
      in_range = (this.GOAL_RANGE_METERS - this.GOAL_RANGE < range && range < this.GOAL_RANGE_METERS + this.GOAL_RANGE);
      SmartDashboard.putBoolean("In Range", in_range);
/*     */ 
/*     */ 
/*     */       
      is_aligned = (-this.GOAL_ALIGNMENT < ((PhotonTrackedTarget)result.getTargets().get(this.targetIndex)).getYaw() + 4.0D && ((PhotonTrackedTarget)result.getTargets().get(this.targetIndex)).getYaw() + 4.0D < this.GOAL_ALIGNMENT);
      SmartDashboard.putBoolean("Is Aligned", is_aligned);
/*     */ 
/*     */       
      if (in_range && is_aligned) {
        in_position = true;
/*     */       } else {
        in_position = false;
/*     */       } 
/*     */     } else {
/*     */       
      in_position = false;
/*     */     } 
/*     */     
    SmartDashboard.putBoolean("In Position", in_position);
/*     */   }
/*     */   
/*     */   public void setLEDState(boolean turnOnYellow) {
    if (turnOnYellow) {
      this.PinkLight.set(ControlMode.PercentOutput, 1.0D);
      this.YellowLight.set(ControlMode.PercentOutput, 0.0D);
/*     */     } else {
      this.PinkLight.set(ControlMode.PercentOutput, 0.0D);
      this.YellowLight.set(ControlMode.PercentOutput, 1.0D);
/*     */     } 
/*     */     
    SmartDashboard.putBoolean("Yellow Target:", turnOnYellow);
    SmartDashboard.putBoolean("Pink Target", !turnOnYellow);
/*     */   }
/*     */   
/*     */   public static boolean isInPosition() {
    return in_position;
/*     */   }
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void aim_GetinRange() {
/*     */     double forwardSpeed, rotationSpeed;
    PhotonPipelineResult resultAT = this.camera.getLatestResult();
/*     */     
    if (resultAT.hasTargets()) {
/*     */       
      double rangeAT = PhotonUtils.calculateDistanceToTargetMeters(this.CAMERA_HEIGHT_METERS, this.TARGET_HEIGHT_METERS, this.CAMERA_PITCH_RADIANS, 
/*     */ 
/*     */ 
/*     */           
          Units.degreesToRadians(resultAT.getBestTarget().getPitch()));
/*     */ 
/*     */ 
/*     */       
      SmartDashboard.putNumber("Current Distance", rangeAT);
      forwardSpeed = -this.forwardController.calculate(rangeAT, this.GOAL_RANGE_METERS);
/*     */ 
/*     */ 
/*     */       
      rotationSpeed = this.turnController.calculate(resultAT.getBestTarget().getYaw(), 0.0D);
/*     */     } else {
/*     */       
      forwardSpeed = 0.0D;
      rotationSpeed = 0.0D;
/*     */     } 
/*     */ 
/*     */ 
/*     */     
    SmartDashboard.putNumber("rotational Speed", rotationSpeed);
    SmartDashboard.putNumber("forwardSpeed", forwardSpeed);
/*     */     
    System.out.println("DRIVER ROBOT VISION!!!");
    
    this.m_drivetrain.visionArcadeDrive(forwardSpeed, rotationSpeed);
/*     */   }
/*     */ }

