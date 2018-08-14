#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <cmath>     // Needed for sin, cos
#include <Eigen/Dense>
#include <iostream>
#include "particle.hpp"
#include "line.hpp"
#define PI 3.14159265

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

// Global variables
char title[] = "Full-Screen & Windowed Mode";  // Windowed mode's title
int windowWidth  = 640;     // Windowed mode's width
int windowHeight = 480;     // Windowed mode's height
int windowPosX   = 50;      // Windowed mode's top-left corner x
int windowPosY   = 50;      // Windowed mode's top-left corner y

int refreshMillis = 30;      // Refresh period in milliseconds
double dt = refreshMillis * 1e-3;
bool applyGravity = true;

double ballMass = 1;
double ballRadius = 0.2;   // Radius of the bouncing ballRadius
double prev_pos[] = {0,0,2}, pos[] = {0,0,2}, vel[] ={0,0,0}, acc[] = {0,0,0};
particle Ball(ballMass, ballRadius, dt, prev_pos, pos, vel, acc);

double start[] = {0,0,-5}, end[] = {0,0,5}, line_length = 10;
line_segment Line0(start, end);

static double org_dist = 10.0, org_pitch = 80.0, org_yaw = 0.0;
double distance = org_dist, pitch = org_pitch, yaw = org_yaw;
int mouse_button = -1;
int mouse_x = 0, mouse_y = 0;

static const GLfloat light_position[] = { 5.0, 5.0, 10.0, 1.0 };
static const GLfloat light_ambient[] = {1.0, 1.0, 1.0, 1.0};
static const GLfloat light_diffuse[] = {0.5, 0.5, 0.5, 1.0};

// Projection clipping area
bool fullScreenMode = false; // Full-screen or windowed mode?

// Color for objects
float red[] = {0.9, 0.1, 0.1, 1.0};
float green[] = {0.1, 0.9, 0.1, 1.0};
float blue[] = {0.1, 0.1, 0.9, 1.0};
float yellow[] = {0.9, 0.9, 0.1, 1.0};

Eigen::Vector3d axis = e2;
double rotate_ang = 1 * dt; // unit: deg/s * s

/* Initialize OpenGL Graphics */
void initGL() {
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
  Ball.init();
  if (applyGravity) {                     // Apply Gravity mode
     Ball.force = - 0.05 * Ball.mass * grav * e3;
  } else {                                // Non-apply Gravity mode
     Ball.force = Eigen::Vector3d::Zero();
  }
}

void physics_calculate(){
   // Rotate line
   double half_length = (Line0.end - Line0.start).norm() * 0.5;
   Eigen::Vector3d line_dir = (Line0.end - Line0.start).normalized();
   Eigen::Vector3d cent = (Line0.end + Line0.start) * 0.5;
   Eigen::Vector3d c_to_s = Line0.start - cent;
   Eigen::Vector3d c_to_e = Line0.end - cent;
   Eigen::Vector3d dstart = axis.cross(c_to_s)/(axis.norm() * c_to_s.norm()) * sin(rotate_ang);
   c_to_s += dstart;
   c_to_s = c_to_s / c_to_s.norm() * half_length;
   Line0.start = cent + c_to_s;
   //
   Eigen::Vector3d dend = axis.cross(c_to_e)/(axis.norm() * c_to_e.norm()) * sin(rotate_ang);
   c_to_e += dend;
   c_to_e = c_to_e / c_to_e.norm() * half_length;
   Line0.end = cent + c_to_e;

   // Move Ball along with line movement
   double pos_on_line = (Ball.pos - cent).dot(line_dir);
   line_dir = (Line0.end - Line0.start).normalized();
   Ball.pos = cent * 0.5 + line_dir * pos_on_line;
   Ball.prev_pos = Ball.pos - Ball.vel * Ball.last_dt;

   // Animation Control - compute the location for the next refresh
   // Ball.updateEuler(dt);
   Ball.updateVerlet(dt);

   // Project to movement to line
   Eigen::Vector3d dx = Ball.pos - Ball.prev_pos;
   dx = dx.dot(line_dir) * line_dir;
   Ball.pos = Ball.prev_pos + dx; // Project to particle movement to line
   Eigen::Vector3d start_to_Ball = Ball.pos - Line0.start;
   double dist_on_line = start_to_Ball.dot(line_dir);
   if (dist_on_line < 0) {
      Ball.pos = Line0.start; // If ball goes over start point of Line, move ball to start point
   } else if (dist_on_line > (Line0.end - Line0.start).norm()){
      Ball.pos = Line0.end; // If ball goes over end point of Line, move ball to end point
   }
}

void draw_line_segment(line_segment Line){
   glLineWidth(5.0);
   glDisable(GL_LIGHTING); glBegin(GL_LINES);
   glColor3f(1, 1, 0); //yellow
   glVertex3d(Line.start[0], Line.start[1], Line.start[2]);
   glVertex3d(Line.end[0], Line.end[1], Line.end[2]);
   glEnd(); glEnable(GL_LIGHTING);
   glLineWidth(1.0);
}

void draw_floor(){
   double l = 10, s = 50;
   glDisable(GL_LIGHTING); glBegin(GL_LINES);
   glColor3f(1, 1, 1);
   for (int y = -s; y <= s; y += l) {
      for (int x = -s; x <= s; x += l) {
         glVertex3d (x, -s, 0);
         glVertex3d (x,  s, 0);
         glVertex3d (-s, y, 0);
         glVertex3d ( s, y, 0);
      }
   }
   glEnd(); glEnable(GL_LIGHTING);
}

void draw_origin(){
   double l = 1.5;
   glDisable(GL_LIGHTING); glBegin(GL_LINES);
   glColor3f(1, 0, 0); glVertex3d(0, 0, 0); glVertex3d(l, 0, 0);
   glColor3f(0, 1, 0); glVertex3d(0, 0, 0); glVertex3d(0, l, 0);
   glColor3f(0, 0, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, l);
   glEnd(); glEnable(GL_LIGHTING);
}

/* Callback handler for window re-paint event */
void display() {
   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the color buffer
   glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
   glLoadIdentity ();             /* clear the matrix */
           /* viewing transformation  */
   gluLookAt(0.0, 0.0, distance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
   glRotatef( -pitch, 1.0, 0.0, 0.0 );
   glRotatef( -yaw, 0.0, 1.0, 0.0 );
   glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
   // draw_floor();
   draw_origin();
   // Line0.rotate(0.5*dt, e2);
   physics_calculate();
   draw_line_segment(Line0);
   //
   // glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);
   glEnable(GL_COLOR_MATERIAL);
   glColor4f(0.9, 0.1, 0.1, 1.0); // red
   glTranslatef(Ball.pos[0], Ball.pos[1], Ball.pos[2]);  // Translate to (xPos, yPos, zPos)
   glutSolidSphere (Ball.radius, 32, 32);
   glDisable(GL_COLOR_MATERIAL);
   glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
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
         Ball.force = - 0.05 * Ball.mass * grav * e3;
      } else {                                // Non-apply Gravity mode
         Ball.force = Eigen::Vector3d::Zero();
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
      Ball.vel[0] += 0.002; break;
   case GLUT_KEY_LEFT:     // Left: decrease x speed
      Ball.vel[0] -= 0.002; break;
   case GLUT_KEY_UP:       // Up: increase y speed
      Ball.vel[1] += 0.002; break;
   case GLUT_KEY_DOWN:     // Down: decrease y speed
      Ball.vel[1] -= 0.002; break;
   case GLUT_KEY_PAGE_UP:  // Page-Up: increase ball's radius
      Ball.radius *= 1.05;
      break;
   case GLUT_KEY_PAGE_DOWN: // Page-Down: decrease ball's radius
      Ball.radius *= 0.95;
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
   initGL();
   glutMainLoop();               // Enter event-processing loop
   return 0;
}
