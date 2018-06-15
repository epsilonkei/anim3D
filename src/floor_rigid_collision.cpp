#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "floor_rigid.hpp"
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
// double dt = 1e-3;
bool applyGravity = true;

#define N_part_per_rigid 8
#define N_rigid 1
double part_mass = 1, part_radius = 0.1, height = 2;
double prev_poss[N_part_per_rigid * N_rigid][3], poss[N_part_per_rigid * N_rigid][3],
   vels[N_part_per_rigid * N_rigid][3], accs[N_part_per_rigid * N_rigid][3];

#define N_floor 1
double floor_elass[N_floor];
double floor_orgs[N_floor][3], floor_norms[N_floor][3];

floor_rigids FLR;

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
   // for rigids
   double rgl = 0.4;
   for (int i = 0; i < N_rigid; i++) {
      for (int j = 0; j < N_part_per_rigid; j++) {
         if (j == 0) {
            poss[i*8+j][0] = 0; poss[i*8+j][1] = 0; poss[i*8+j][2] = height;
         } else if (j == 1) {
            poss[i*8+j][0] = 0; poss[i*8+j][1] = rgl; poss[i*8+j][2] = height;
         } else if (j == 2) {
            poss[i*8+j][0] = rgl; poss[i*8+j][1] = rgl; poss[i*8+j][2] = height;
         } else if (j == 3) {
            poss[i*8+j][0] = rgl; poss[i*8+j][1] = 0; poss[i*8+j][2] = height;
         } else if (j == 4) { //
            poss[i*8+j][0] = 0; poss[i*8+j][1] = 0; poss[i*8+j][2] = height + rgl;
         } else if (j == 5) {
            poss[i*8+j][0] = 0; poss[i*8+j][1] = rgl; poss[i*8+j][2] = height + rgl;
         } else if (j == 6) {
            poss[i*8+j][0] = rgl; poss[i*8+j][1] = rgl; poss[i*8+j][2] = height + rgl;
         } else if (j == 7) {
            poss[i*8+j][0] = rgl; poss[i*8+j][1] = 0; poss[i*8+j][2] = height + rgl;
         }
         prev_poss[i*8+j][0] = poss[i*8+j][0];
         prev_poss[i*8+j][1] = poss[i*8+j][1];
         prev_poss[i*8+j][2] = poss[i*8+j][2];
         vels[i*8+j][0] = 0; vels[i*8+j][1] = 0; vels[i*8+j][2] = 0;
         accs[i*8+j][0] = 0; accs[i*8+j][1] = 0; accs[i*8+j][2] = 0;
         FLR.add_particle(part_mass, part_radius, dt, prev_poss[i*8+j], poss[i*8+j],
                          vels[i*8+j], accs[i*8+j]);
      }
   }
   for (int i = 0; i < FLR.rl[0]->pl.size(); i++) {
      FLR.rl[0]->pl[i]->init();
      Eigen::Vector3d tmp (0.1, 0.2, 0);
      FLR.rl[0]->omega = tmp;
      FLR.rl[0]->pl[i]->force = - FLR.rl[0]->pl[i]->mass * grav * e3;
   }
   FLR.rl[0]->init();
   // for floor
   floor_elass[0] = 1;
   floor_orgs[0][0] = floor_orgs[0][1] = floor_orgs[0][2] = 0;
   floor_norms[0][0] = floor_norms[0][1] = 0; floor_norms[0][2] = 1;
   FLR.add_floor(floor_orgs[0], floor_norms[0], floor_elass[0]);
}

void physics_calculate(){
   FLR.update_movement(dt);
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
   draw_floor();
   draw_origin();
   // Draw particle
   glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);
   for (int i = 0; i < FLR.rl[0]->pl.size(); i++) {
      glPushMatrix();
      glTranslatef(FLR.rl[0]->pl[i]->pos[0], FLR.rl[0]->pl[i]->pos[1], FLR.rl[0]->pl[i]->pos[2]);
      glutSolidSphere (FLR.rl[0]->pl[i]->radius, 16, 16);
      glPopMatrix();
   }   glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
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
