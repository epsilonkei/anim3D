#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <cmath>     // Needed for sin, cos
#include <Eigen/Dense>
#include <iostream>
// #include "particle.hpp"
#include "particles.hpp"
#define PI 3.14159265

extern double grav;
extern Eigen::Vector3d e1, e2, e3;
double sum_time = 0;
int N_iterate = 0;

// Global variables
extern double G_const;
char title[] = "Full-Screen & Windowed Mode";  // Windowed mode's title
int windowWidth  = 640;     // Windowed mode's width
int windowHeight = 480;     // Windowed mode's height
int windowPosX   = 50;      // Windowed mode's top-left corner x
int windowPosY   = 50;      // Windowed mode's top-left corner y

int refreshMillis = 30;      // Refresh period in milliseconds
double dt = refreshMillis * 1e-3;
bool applyGravity = false;

double table_length = 10;
particles PL(table_length);

#define N_ball 100
double prev_poss[N_ball][3], poss[N_ball][3], vels[N_ball][3], accs[N_ball][3];

static double org_dist = 19.0, org_pitch = 10.0, org_yaw = 0.0;
double distance = org_dist, pitch = org_pitch, yaw = org_yaw;
int mouse_button = -1;
int mouse_x = 0, mouse_y = 0;

static const GLfloat light_position[] = { 5.0, 5.0, 10.0, 1.0 };
static const GLfloat light_ambient[] = {1.0, 1.0, 1.0, 1.0};
static const GLfloat light_diffuse[] = {0.5, 0.5, 0.5, 1.0};

bool fullScreenMode = false; // Full-screen or windowed mode?

// Color for objects
float red[] = {0.9, 0.1, 0.1, 1.0};
float green[] = {0.1, 0.9, 0.1, 1.0};
float blue[] = {0.1, 0.1, 0.9, 1.0};

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

}

void initSim() {
   double tbl = PL.table_length;
   // double ball_radiuss = 0.1, ball_masss = 1;
   double ball_radiuss = std::min(0.2, tbl * 10/N_ball), ball_masss = 1;
   srand(0);
   for (uint i = 0; i < N_ball; i++) {
      // prev_poss[i][0] = i * 0.5 - 3; prev_poss[i][1] = 0; prev_poss[i][2] = ball_radiuss;
      poss[i][0] = double(rand()) / RAND_MAX * 2 * tbl - tbl;
      poss[i][1] = double(rand()) / RAND_MAX * 2 * tbl - tbl;
      poss[i][2] = ball_radiuss;
      prev_poss[i][0] = poss[i][0]; prev_poss[i][1] = poss[i][1]; prev_poss[i][2] = poss[i][2];
      vels[i][0] = 0; vels[i][1] = 0; vels[i][2] = 0;
      accs[i][0] = 0; accs[i][1] = 0; accs[i][2] = 0;
      // accs[i] = {0,0,0}; only works with C++0x and above
      PL.add_particle(ball_masss, ball_radiuss, dt, prev_poss[i], poss[i], vels[i], accs[i]);
   }
   for (uint i = 0; i < PL.pl.size(); i++) {
      PL.pl[i]->init();
   }
}

struct timespec startTime, endTime;

void physics_calculate(){
   // Animation Control - compute the location for the next Refresh
   // PL.updateWithGravitationalForce(dt);
   clock_gettime(CLOCK_REALTIME, &startTime);
   //
   PL.setGravitationalForceAllWithBHAlg();
   //
   clock_gettime(CLOCK_REALTIME, &endTime);
   sum_time += (double)(endTime.tv_sec - startTime.tv_sec + (endTime.tv_nsec - startTime.tv_nsec) * 1e-9);
   N_iterate += 1;
   //
   PL.updateVerletAll(dt);
   PL.checkBounceCollideAll();
   // if (N_iterate > 20) {
   //    std::cout << "Average time: " << (sum_time / N_iterate) << " [sec]" << std::endl;
   //    exit(0);
   // }
}

void draw_particles(particles _PL){
   double tbl = _PL.table_length, h_side = 1;
   // Table
   glDisable(GL_LIGHTING); glBegin(GL_QUADS);
   glColor3f(0.5, 0.5, 0.5);
   glVertex3f ( tbl, -tbl, 0);
   glVertex3f ( tbl,  tbl, 0);
   glVertex3f (-tbl,  tbl, 0);
   glVertex3f (-tbl, -tbl, 0);
   //
   glColor3f(0.2, 0.2, 0.2);
   glVertex3f ( tbl, -tbl, 0);
   glVertex3f ( tbl,  tbl, 0);
   glVertex3f ( tbl,  tbl, h_side);
   glVertex3f ( tbl, -tbl, h_side);
   //
   glVertex3f ( tbl,  tbl, 0);
   glVertex3f (-tbl,  tbl, 0);
   glVertex3f (-tbl,  tbl, h_side);
   glVertex3f ( tbl,  tbl, h_side);
   //
   glVertex3f (-tbl,  tbl, 0);
   glVertex3f (-tbl, -tbl, 0);
   glVertex3f (-tbl, -tbl, h_side);
   glVertex3f (-tbl,  tbl, h_side);
   //
   glVertex3f (-tbl, -tbl, 0);
   glVertex3f ( tbl, -tbl, 0);
   glVertex3f ( tbl, -tbl, h_side);
   glVertex3f (-tbl, -tbl, h_side);
   //
   glEnd(); glEnable(GL_LIGHTING);
   // Draw ball
   glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);
   for (uint i = 0; i < _PL.pl.size(); i++) {
      glPushMatrix();
      // glTranslatef(_PL.pl[i].pos[0], _PL.pl[i].pos[1], _PL.pl[i].pos[2]);
      glTranslatef(_PL.pl[i]->pos[0], _PL.pl[i]->pos[1], _PL.pl[i]->pos[2]);
      glutSolidSphere (_PL.pl[i]->radius, 16, 16);
      glPopMatrix();
   }
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
   gluLookAt(0.0, 0.0, distance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
   glRotatef( -pitch, 1.0, 0.0, 0.0 );
   glRotatef( -yaw, 0.0, 1.0, 0.0 );
   physics_calculate();
   // draw_floor();
   draw_origin();
   draw_particles(PL);
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
      // std::cout << "Average time: " << (sum_time / N_iterate) << " [sec]" << std::endl;
      exit(0);
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
   initSim();
   initGL();
   glutMainLoop();               // Enter event-processing loop
   return 0;
}
