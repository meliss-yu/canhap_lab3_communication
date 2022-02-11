/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
float count=0;
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox              b1;
FBox              b2;
FBox              b3;
FBox              b4;
FBox              b5;
FBox              b6;
FBox              b7;
FBox          b8;
FBox b9;
FBox b10;

FCircle c1;
FCircle c2; 
FCircle c3; 
FCircle c4; 
FCircle c5;
FCircle c6; 
FCircle c7;
FCircle c8;


/* define game ball */
FCircle           g2;
FBox              g1;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  ///* Set maze barriers */
  b1                  = new FBox(5.0, 5.0);
  b1.setPosition(edgeTopLeftX+worldWidth/4.0-2, edgeTopLeftY+worldHeight/2); 
  b1.setStroke(0);
  //b1.setFill(150,150,255,80);
  b1.setSensor(true);
  b1.setStrokeWeight(5);
  b1.setStroke(0);
  b1.setStatic(true);
  world.add(b1);
  
  b2                  = new FBox(5.0, 5.0);
  b2.setPosition(edgeTopLeftX+worldWidth/4.0+6, edgeTopLeftY+worldHeight/2); 
  b2.setStroke(0);
  b2.setStrokeWeight(5);
  b2.setStatic(true);
  b2.setSensor(true);
  world.add(b2);
   
  b3                  = new FBox(1.25, 1.25);
  b3.setPosition(edgeTopLeftX+worldWidth/4.0+4.5, edgeTopLeftY+worldHeight/2-1.5); 
  b3.setNoStroke();
  b3.setStatic(true);
  b3.setSensor(true);
  world.add(b3);
  
  b4                  = new FBox(1.25, 1.25);
  b4.setPosition(edgeTopLeftX+worldWidth/4.0+7.5, edgeTopLeftY+worldHeight/2-1.5); 
  b4.setNoStroke();
  b4.setStatic(true);
  b4.setSensor(true);
  world.add(b4);
   
  b5                  = new FBox(1.25, 1.25);
  b5.setPosition(edgeTopLeftX+worldWidth/4.0+7.5, edgeTopLeftY+worldHeight/2.0+1.5);
  b5.setNoStroke();
  b5.setStatic(true);
  b5.setSensor(true);
  world.add(b5);
  
  b6                  = new FBox(1.25, 1.25);
  b6.setPosition(edgeTopLeftX+worldWidth/4.0+4.5, edgeTopLeftY+worldHeight/2.0+1.5);
  b6.setNoStroke();
  b6.setStatic(true);
  b6.setSensor(true);
  world.add(b6);
  
  b7                  = new FBox(5.0, 0.1);
  b7.setPosition(edgeTopLeftX+worldWidth/4.0+14, edgeTopLeftY+worldHeight/2+2.5); 
  b7.setStroke(0);
  b7.setStaticBody(true);
  b7.setFill(0);
  world.add(b7);

  
  b8                  = new FBox(3.6, 0.1);
  b8.setPosition(edgeTopLeftX+worldWidth/4.0+13.3, edgeTopLeftY+worldHeight/2-2.5); 
  b8.setStroke(0);
  b8.setFill(0);
  b8.setStaticBody(true);
  //b7.setSensor(true);
  world.add(b8);
  
  b9                  = new FBox(0.1, 5.0);
  b9.setPosition(edgeTopLeftX+worldWidth/4.0+11.5, edgeTopLeftY+worldHeight/2); 
  b9.setStroke(0);
  b9.setStaticBody(true);
  b9.setFill(0);
  world.add(b9);
  
  b10                 = new FBox(0.1, 5.0);
  b10.setPosition(edgeTopLeftX+worldWidth/4.0+16.5, edgeTopLeftY+worldHeight/2); 
  b10.setStroke(0);
  b10.setStaticBody(true);
  b10.setFill(0);
  world.add(b10);
  
  c1 = new FCircle(1.5);
  c1.setPosition(edgeTopLeftX+worldWidth/4.0+14, edgeTopLeftY+worldHeight/2+1);
  c1.setDensity(10);
  c1.setNoStroke();
  world.add(c1);
  
  c2 = new FCircle(1.5);
  c2.setPosition(edgeTopLeftX+worldWidth/4.0+15, edgeTopLeftY+worldHeight/2+1);
  c2.setDensity(10);
  c2.setNoStroke();
  world.add(c2);
  
  c3 = new FCircle(1.5);
  c3.setPosition(edgeTopLeftX+worldWidth/4.0+15, edgeTopLeftY+worldHeight/2);
  c3.setDensity(10);
  c3.setNoStroke();
  world.add(c3);
   
  c4 = new FCircle(1.5);
  c4.setPosition(edgeTopLeftX+worldWidth/4.0+14, edgeTopLeftY+worldHeight/2+1);
  c4.setDensity(3);
  c4.setNoStroke();
  world.add(c4);
  
  c5 = new FCircle(1.5);
  c5.setPosition(edgeTopLeftX+worldWidth/4.0+13.2, edgeTopLeftY+worldHeight/2+1);
  c5.setDensity(10);
  c5.setNoStroke();
  world.add(c5);
   
  c6 = new FCircle(1.5);
  c6.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2+1);
  c6.setDensity(10);
  c6.setNoStroke();
  world.add(c6);
  
  c7 = new FCircle(1.5);
  c7.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2+1);
  c7.setDensity(6);
  c7.setNoStroke();
  world.add(c7);
  
  c8 = new FCircle(1.5);
  c8.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2+1);
  c8.setDensity(15);
  c8.setNoStroke();
  world.add(c8);
   
   
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(false);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
 // world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  

  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    //if((posEE.y < b7.getY()-2.5) && (posEE.y > b7.getY()+2.5)){
    //  if((posEE.x > (b7.getX()-2.5)) && (posEE.x < (b7.getX()))){
    //    fEE.set(5,0);
    //  } else if ((posEE.x > b7.getX()) && (posEE.x < (b7.getX()+2.5))){
    //    fEE.set(-5, 0);
    //  } else {
    //    fEE.set(0,0);
    //  }
    //}
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    //if (s.h_avatar.isTouchingBody(c1)){
    //  gameStart = true;
    //  g1.setPosition(2,8);
    //  g2.setPosition(3,8);
    //  s.h_avatar.setSensor(false);
    //}
  
    //if(g1.isTouchingBody(c2) || g2.isTouchingBody(c2)){
    //  gameStart = false;
    //  s.h_avatar.setSensor(true);
    //}
  
  
  
    /* Viscous layer codes */

   if(s.h_avatar.isTouchingBody(b1)){
     s.h_avatar.setDamping(600);
   } else {
     s.h_avatar.setDamping(10);
   }
   
   if(count<14){
     if(s.h_avatar.isTouchingBody(b3)){
       s.h_avatar.adjustVelocity(0.01, 0.01);
       s.h_avatar.setPosition(b4.getX(), b4.getY());
       count = count+1;
       //delay(100);
     }
     if(s.h_avatar.isTouchingBody(b4)){
       s.h_avatar.adjustVelocity(0.01, 0.01);
       s.h_avatar.setPosition(b5.getX(), b5.getY());
       count = count+1;
       //delay(100);
     }
     if(s.h_avatar.isTouchingBody(b5)){
       s.h_avatar.adjustVelocity(0.01, 0.01);
       s.h_avatar.setPosition(b6.getX(), b6.getY());
       count = count+1;
       //delay(100);
     }
     if(s.h_avatar.isTouchingBody(b6)){
       s.h_avatar.adjustVelocity(0.01, 0.01);
       //s.h_avatar.setPosition(edgeTopLeftX+worldWidth/4.0+10.5, edgeTopLeftY+worldHeight/2.0+1.5);
       count = count+1; 
       s.h_avatar.setPosition(b3.getX(), b3.getY());
       //delay(100);
     }
   } else {
     if(s.h_avatar.getX()<(edgeTopLeftX+worldWidth/4.0+6)){
       s.h_avatar.setPosition(b1.getX()+3,b1.getY());
     } else {
       s.h_avatar.setPosition(edgeTopLeftX+worldWidth/4.0+10, edgeTopLeftY+worldHeight/2.0+1.5);
       
     }
     count=0;
   }
   if (s.h_avatar.isTouchingBody(b7)){
   //  if(s.h_avatar.getX() < (edgeTopLeftX+worldWidth/4.0+14)){
   //    s.h_avatar.addForce(10, 0);
   //  } else if (s.h_avatar.getX() > (edgeTopLeftX+worldWidth/4.0+14)){
   //    s.h_avatar.addForce(-10, 0);
   //  }

   }
  
    /* Bouyancy of fluid on avatar and gameball section */
    //if (g1.isTouchingBody(l1)){
    //  float b_s;
    //  float bm_d = g1.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water
    
    //  if (bm_d + g1.getWidth()/2 >= g1.getWidth()) { //if whole ball or more is submerged
    //    b_s = g1.getWidth(); // amount of ball submerged is ball size
    //  }
    //  else { //if ball is partially submerged
    //    b_s = bm_d + g1.getWidth()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
    //  }
  
    //  g1.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
   
    //}
  
    //if (g2.isTouchingBody(l1)){
    //  float b_s;
    //  float bm_d = g2.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water
    
    //  if (bm_d + g2.getSize()/2 >= g2.getSize()) { //if whole ball or more is submerged
    //    b_s = g2.getSize(); // amount of ball submerged is ball size
    //  }
    //  else { //if ball is partially submerged
    //    b_s = bm_d + g2.getSize()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
    //  }
  
    //  g2.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
     
    //}

    /* End Bouyancy of fluid on avatar and gameball section */
    s.h_avatar.addForce(0, gravityAcceleration);
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
