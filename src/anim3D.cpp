#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <cmath>     // Needed for sin, cos
#include <Eigen/Dense>
#include <iostream>
#include "particle.hpp"
#include "floor.hpp"
#define PI 3.14159265

// Global variables
char title[] = "Full-Screen & Windowed Mode";  // Windowed mode's title
int windowWidth  = 640;     // Windowed mode's width
int windowHeight = 480;     // Windowed mode's height
int windowPosX   = 50;      // Windowed mode's top-left corner x
int windowPosY   = 50;      // Windowed mode's top-left corner y

int refreshMillis = 30;      // Refresh period in milliseconds
double dt = refreshMillis * 1e-3;
bool applyGravity = false;

double ballMass = 1;
double ballRadius = 0.2;   // Radius of the bouncing ballRadius
double grav = -9.8;
double prev_pos[] = {0,0,5}, pos[] = {0,0,5}, vel[] ={0,0,0}, acc[] = {0,0,0};
particle Ball(ballMass, ballRadius, prev_pos, pos, vel, acc);

double floor_elas = 0.5;
double floor_org[] = {0,0,0}, floor_norm[] = {0,0,1};
static_floor Floor0(floor_org, floor_norm, floor_elas);

// Eigen::Vector2d pos(0, 0); // Ball's center (x, y) position
// Eigen::Vector2d vel(0.02, 0.007); // Ball's speed in x and y directions
// Eigen::Vector2d gravity_acc(0, 0); // gravity acceleration
double ballXMax, ballXMin, ballYMax, ballYMin; // Ball's center (x, y) bounds
static double org_dist = 10.0, org_pitch = 20.0, org_yaw = 0.0;
double distance = org_dist, pitch = org_pitch, yaw = org_yaw;
int mouse_button = -1;
int mouse_x = 0, mouse_y = 0;

static const GLfloat light_position[] = { 0.0, 0.0, 10.0, 1.0 };
static const GLfloat light_ambient[] = {1.0, 1.0, 1.0, 1.0};
static const GLfloat light_diffuse[] = {0.5, 0.5, 0.5, 1.0};

static const GLfloat mat_default_color[] = { 1.0, 1.0, 1.0, 1.0 };
static const GLfloat mat_default_specular[] = { 0.0, 0.0, 0.0, 0.0 };
static const GLfloat mat_default_shininess[] = { 100.0 };
static const GLfloat mat_default_emission[] = {0.0, 0.0, 0.0, 0.0};

// Projection clipping area
GLdouble clipAreaXLeft, clipAreaXRight, clipAreaYBottom, clipAreaYTop;
bool fullScreenMode = false; // Full-screen or windowed mode?

/* Initialize OpenGL Graphics */
void initGL() {
  // glClearColor (1.0, 1.0, 1.0, 1.0);
  glClearColor (0.0, 0.0, 0.0, 1.0);
  glClearDepth( 1.0 );

  // Depth Test
  glEnable( GL_DEPTH_TEST );
  glDepthFunc( GL_LESS );

  glShadeModel (GL_SMOOTH);

  // Default light
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_ambient);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // Default material
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_default_color);
  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_default_color);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_default_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_default_shininess);
}

void physics_calculate(){
   // Animation Control - compute the location for the next refresh
   // Ball.updateEuler(dt);
   Ball.updateVelvet(dt);
   // Check if the ball exceeds the edges
   if (Floor0.norm_vec.dot(Ball.pos - Floor0.origin) < Ball.radius &&
       Floor0.norm_vec.dot(Ball.vel) < 0) {
      Ball.vel[2] = - Ball.vel[2] * Floor0.elasticity;
      Ball.pos += Floor0.norm_vec * Ball.radius;
   }
   // if (pos[0] > ballXMax) {
   //    pos[0] = ballXMax;
   //    vel[0] = -vel[0];
   // } else if (pos[0] < ballXMin) {
   //    pos[0] = ballXMin;
   //    vel[0] = -vel[0];
   // }
   // if (pos[1] > ballYMax) {
   //    pos[1] = ballYMax;
   //    vel[1] = -vel[1];
   // } else if (pos[1] < ballYMin) {
   //    pos[1]  = ballYMin;
   //    vel[1] = - vel[1];
   // }

}

void draw_floor(){
   double l = 10, s = 50;
   glDisable(GL_LIGHTING); glBegin(GL_LINES);
   glColor3f(1, 1, 1);
   for (int y = -s; y <= s; y += l) {
      for (int x = -s; x <= s; x += l) {
         glVertex3f (x, -s, 0);
         glVertex3f (x,  s, 0);
         glVertex3f (-s, y, 0);
         glVertex3f ( s, y, 0);
      }
   }
   glEnd(); glEnable(GL_LIGHTING);
}

void draw_origin(){
   double l = 1.5;
   glDisable(GL_LIGHTING); glBegin(GL_LINES);
   glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(l, 0, 0);
   glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, l, 0);
   glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, l);
   glEnd(); glEnable(GL_LIGHTING);
}

/* Callback handler for window re-paint event */
void display() {
   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the color buffer
   glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
   glLoadIdentity ();             /* clear the matrix */
           /* viewing transformation  */
   // gluLookAt (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
   gluLookAt(0.0, 0.0, distance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
   glRotatef( -pitch, 1.0, 0.0, 0.0 );
   glRotatef( -yaw, 0.0, 1.0, 0.0 );
   draw_floor();
   draw_origin();
   // glScalef (1.0, 1.0, 1.0);      /* modeling transformation */

   //glDisable(GL_LIGHTING);
   //glColor3f(1.0, 0.0, 0.0);
   //glTranslatef(0, 0, 5);  // Translate to (xPos, yPos, zPos)
   //std::cout << Ball.pos[0] << ", " << Ball.pos[1] << ", " << Ball.pos[2] << std::endl;
   glTranslatef(Ball.pos[0], Ball.pos[1], Ball.pos[2]);  // Translate to (xPos, yPos, zPos)
   glutSolidSphere (ballRadius, 16, 16);
   //glEnable(GL_LIGHTING);
   glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
   physics_calculate();
}

/* Call back when the windows is re-sized */
void reshape(GLsizei width, GLsizei height) {
   // Compute aspect ratio of the new window
   if (height == 0) height = 1;                // To prevent divide by 0
   GLfloat aspect = (GLfloat)width / (GLfloat)height;

   // Set the viewport to cover the new window
   glViewport(0, 0, width, height);

   // Set the aspect ratio of the clipping area to match the viewport
   glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
   glLoadIdentity();             // Reset the projection matrix
   gluPerspective(60.0, aspect, 1.0, 60.0);
   if (width >= height) {
      clipAreaXLeft   = -1.0 * aspect;
      clipAreaXRight  = 1.0 * aspect;
      clipAreaYBottom = -1.0;
      clipAreaYTop    = 1.0;
   } else {
      clipAreaXLeft   = -1.0;
      clipAreaXRight  = 1.0;
      clipAreaYBottom = -1.0 / aspect;
      clipAreaYTop    = 1.0 / aspect;
   }
   // gluOrtho2D(clipAreaXLeft, clipAreaXRight, clipAreaYBottom, clipAreaYTop);
   ballXMin = clipAreaXLeft + ballRadius;
   ballXMax = clipAreaXRight - ballRadius;
   ballYMin = clipAreaYBottom + ballRadius;
   ballYMax = clipAreaYTop - ballRadius;
}

/* Called back when the timer expired */
void Timer(int value) {
   glutPostRedisplay();    // Post a paint request to activate display()
   glutTimerFunc(refreshMillis, Timer, 0); // subsequent timer call at milliseconds
}

/* Callback handler for normal-key event */
void keyboard(unsigned char key, int x, int y) {
   switch (key) {
   case 27:     // ESC key
      exit(0);
      break;
   case 'g':    // g: Apply gravity mode
      applyGravity = !applyGravity;         // Toggle state
      if (applyGravity) {                     // Apply Gravity mode
         Ball.acc[2] = grav;
      } else {                                // Non-apply Gravity mode
         Ball.acc[1] = 0;
      }
      break;
   case 'r':    // r: Reset default camera
      distance = org_dist, pitch = org_pitch, yaw = org_yaw;
      break;
   }
}

/* Callback handler for special-key event */
void specialKeys(int key, int x, int y) {
   switch (key) {
   case GLUT_KEY_F1:    // F1: Toggle between full-screen and windowed mode
      fullScreenMode = !fullScreenMode;         // Toggle state
      if (fullScreenMode) {                     // Full-screen mode
         windowPosX   = glutGet(GLUT_WINDOW_X); // Save parameters for restoring later
         windowPosY   = glutGet(GLUT_WINDOW_Y);
         windowWidth  = glutGet(GLUT_WINDOW_WIDTH);
         windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
         glutFullScreen();                      // Switch into full screen
      } else {                                         // Windowed mode
         glutReshapeWindow(windowWidth, windowHeight); // Switch into windowed mode
         glutPositionWindow(windowPosX, windowPosX);   // Position top-left corner
      }
      break;
   case GLUT_KEY_RIGHT:    // Right: increase x speed
      vel[0] += 0.002; break;
   case GLUT_KEY_LEFT:     // Left: decrease x speed
      vel[0] -= 0.002; break;
   case GLUT_KEY_UP:       // Up: increase y speed
      vel[1] += 0.002; break;
   case GLUT_KEY_DOWN:     // Down: decrease y speed
      vel[1] -= 0.002; break;
   case GLUT_KEY_PAGE_UP:  // Page-Up: increase ball's radius
      ballRadius *= 1.05;
      ballXMin = clipAreaXLeft + ballRadius;
      ballXMax = clipAreaXRight - ballRadius;
      ballYMin = clipAreaYBottom + ballRadius;
      ballYMax = clipAreaYTop - ballRadius;
      break;
   case GLUT_KEY_PAGE_DOWN: // Page-Down: decrease ball's radius
      ballRadius *= 0.95;
      ballXMin = clipAreaXLeft + ballRadius;
      ballXMax = clipAreaXRight - ballRadius;
      ballYMin = clipAreaYBottom + ballRadius;
      ballYMax = clipAreaYTop - ballRadius;
      break;
   }
}

/* Callback handler for mouse event */
void mouse(int button, int state, int x, int y)
{
  mouse_button = button;
  mouse_x = x; mouse_y = y;
  if(state == GLUT_UP){
    mouse_button = -1;
  }
  glutPostRedisplay();
}

void motion(int x, int y)
{
  switch(mouse_button){
  case GLUT_LEFT_BUTTON:
    if( x == mouse_x && y == mouse_y )
      return;
    yaw -= (GLfloat) (x - mouse_x) / 2.0;
    pitch -= (GLfloat) (y - mouse_y) / 2.0;
    break;
  case GLUT_RIGHT_BUTTON:
    if( y == mouse_y )
      return;
    if( y < mouse_y )
      distance += (GLfloat) (mouse_y - y)/50.0;
    else
      distance -= (GLfloat) (y - mouse_y)/50.0;
    if( distance < 1.0 ) distance = 1.0;
    if( distance > 50.0 ) distance = 50.0;
    break;
  }
  mouse_x = x;
  mouse_y = y;
  glutPostRedisplay();
}

void idle()
{
  glutPostRedisplay();
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {
   glutInit(&argc, argv);            // Initialize GLUT
   glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH ); // Enable double buffered mode
   glutInitWindowSize(windowWidth, windowHeight);  // Initial window width and height
   glutInitWindowPosition(windowPosX, windowPosY); // Initial window top-left corner (x, y)
   glutCreateWindow(title);      // Create window with given title
   glutDisplayFunc(display);     // Register callback handler for window re-paint
   glutReshapeFunc(reshape);     // Register callback handler for window re-shape
   glutTimerFunc(0, Timer, 0);   // First timer call immediately
   glutSpecialFunc(specialKeys); // Register callback handler for special-key event
   glutKeyboardFunc(keyboard);   // Register callback handler for special-key event
   glutIdleFunc(idle);
   glutMouseFunc(mouse);
   glutMotionFunc(motion);
   if (fullScreenMode) {
      glutFullScreen();             // Put into full screen
   } else {
      glutReshapeWindow(windowWidth, windowHeight); // Switch into windowed mode
   }
   initGL();                     // Our own OpenGL initialization
   glutMainLoop();               // Enter event-processing loop
   return 0;
}
