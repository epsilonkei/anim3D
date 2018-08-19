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
double dt = refreshMillis * 1e-4;
// double dt = 1e-3;

#define table_length 5
#define rigid_size 0.4

#define N_part_per_rigid 8
#define N_rigid 15
double part_mass = 1, part_radius = 0.1, rigid_height = 0.1;
double prev_poss[N_part_per_rigid * N_rigid][3], poss[N_part_per_rigid * N_rigid][3],
   vels[N_part_per_rigid * N_rigid][3], accs[N_part_per_rigid * N_rigid][3];

#define N_floor 5
double floor_elass[N_floor];
double floor_orgs[N_floor][3], floor_norms[N_floor][3];

floor_rigids FLR;

static double org_dist = 11.0, org_pitch = 20.0, org_yaw = 0.0;
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

float color[][4] = {{0.9, 0.1, 0.1, 1.0}, // red
                    {0.1, 0.9, 0.1, 1.0}, // green
                    {0.1, 0.1, 0.9, 1.0}, // blue
                    {0.9, 0.9, 0.1, 1.0}, // yellow
                    {0.9, 0.1, 0.9, 1.0}, // magenta
                    {0.1, 0.9, 0.9, 1.0}};// cyan

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

void initRigids() {
   // for rigids
   srand(0);
   double com_x, com_y, rgl = rigid_size;
   for (uint i = 0; i < N_rigid; i++) {
      com_x = double(rand()) / RAND_MAX * 2 * table_length - table_length;
      com_y = double(rand()) / RAND_MAX * 2 * table_length - table_length;
      for (uint j = 0; j < N_part_per_rigid; j++) {
         if (j == 0) {
            poss[i*8+j][0] = com_x - 0.5*rgl; poss[i*8+j][1] = com_y - 0.5*rgl;
            poss[i*8+j][2] = rigid_height;
         } else if (j == 1) {
            poss[i*8+j][0] = com_x - 0.5*rgl; poss[i*8+j][1] = com_y + 0.5*rgl;
            poss[i*8+j][2] = rigid_height;
         } else if (j == 2) {
            poss[i*8+j][0] = com_x + 0.5*rgl; poss[i*8+j][1] = com_y + 0.5*rgl;
            poss[i*8+j][2] = rigid_height;
         } else if (j == 3) {
            poss[i*8+j][0] = com_x + 0.5*rgl; poss[i*8+j][1] = com_y - 0.5*rgl;
            poss[i*8+j][2] = rigid_height;
         } else if (j == 4) { //
            poss[i*8+j][0] = com_x - 0.5*rgl; poss[i*8+j][1] = com_y - 0.5*rgl;
            poss[i*8+j][2] = rigid_height + rgl;
         } else if (j == 5) {
            poss[i*8+j][0] = com_x - 0.5*rgl; poss[i*8+j][1] = com_y + 0.5*rgl;
            poss[i*8+j][2] = rigid_height + rgl;
         } else if (j == 6) {
            poss[i*8+j][0] = com_x + 0.5*rgl; poss[i*8+j][1] = com_y + 0.5*rgl;
            poss[i*8+j][2] = rigid_height + rgl;
         } else if (j == 7) {
            poss[i*8+j][0] = com_x + 0.5*rgl; poss[i*8+j][1] = com_y - 0.5*rgl;
            poss[i*8+j][2] = rigid_height + rgl;
         }
         prev_poss[i*8+j][0] = poss[i*8+j][0];
         prev_poss[i*8+j][1] = poss[i*8+j][1];
         prev_poss[i*8+j][2] = poss[i*8+j][2];
         vels[i*8+j][0] = 0; vels[i*8+j][1] = 0; vels[i*8+j][2] = 0;
         accs[i*8+j][0] = 0; accs[i*8+j][1] = 0; accs[i*8+j][2] = 0;
         FLR.add_particle(part_mass, part_radius, dt, prev_poss[i*8+j], poss[i*8+j],
                          vels[i*8+j], accs[i*8+j], i);
      }
   }
}

void initSim() {
   initRigids();
   for (uint i = 0; i < FLR.rl.size(); i++) {
      for (uint j = 0; j < FLR.rl[i]->pl.size(); j++) {
         FLR.rl[i]->pl[j]->init();
         // FLR.rl[0]->pl[i]->force = - FLR.rl[0]->pl[i]->mass * grav * e3;
      }
      Eigen::Vector3d tmp_omega (0.2, 0.1, 0.1), tmp_vel(2, 0.5, 0);
      FLR.rl[i]->length = rigid_size;
      FLR.rl[i]->omega = tmp_omega;
      FLR.rl[i]->vel = tmp_vel;
      FLR.rl[i]->init();
   }
   // for floor
   floor_elass[0] = 1;
   floor_orgs[0][0] = floor_orgs[0][1] = floor_orgs[0][2] = 0;
   floor_norms[0][0] = floor_norms[0][1] = 0; floor_norms[0][2] = 1;
   FLR.add_floor(floor_orgs[0], floor_norms[0], floor_elass[0]);
   // for side wall
   floor_elass[1] = 1;
   floor_orgs[1][0] = -table_length;floor_orgs[1][1] = 0;floor_orgs[1][2] = 0;
   floor_norms[1][0] = 1;floor_norms[1][1] = 0; floor_norms[1][2] = 0;
   FLR.add_floor(floor_orgs[1], floor_norms[1], floor_elass[1]);
   //
   floor_elass[2] = 1;
   floor_orgs[2][0] = table_length;floor_orgs[2][1] = 0;floor_orgs[2][2] = 0;
   floor_norms[2][0] = -1;floor_norms[2][1] = 0; floor_norms[2][2] = 0;
   FLR.add_floor(floor_orgs[2], floor_norms[2], floor_elass[2]);
   //
   floor_elass[3] = 1;
   floor_orgs[3][0] = 0;floor_orgs[3][1] = -table_length;floor_orgs[3][2] = 0;
   floor_norms[3][0] = 0;floor_norms[3][1] = 1; floor_norms[3][2] = 0;
   FLR.add_floor(floor_orgs[3], floor_norms[3], floor_elass[3]);
   //
   floor_elass[4] = 1;
   floor_orgs[4][0] = 0;floor_orgs[4][1] = table_length;floor_orgs[4][2] = 0;
   floor_norms[4][0] = 0;floor_norms[4][1] = -1; floor_norms[4][2] = 0;
   FLR.add_floor(floor_orgs[4], floor_norms[4], floor_elass[4]);
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

void draw_rigids(){
   double tbl = table_length, h_side = 1;
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
   // Draw rigids
   for (uint i = 0; i<FLR.rl.size(); i++) {
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color[i % (sizeof(color)/sizeof(*color))]);
      for (uint j = 0; j < FLR.rl[i]->pl.size(); j++) {
         glPushMatrix();
         glTranslatef(FLR.rl[i]->pl[j]->pos[0], FLR.rl[i]->pl[j]->pos[1], FLR.rl[i]->pl[j]->pos[2]);
         glutSolidSphere (FLR.rl[i]->pl[j]->radius, 16, 16);
         glPopMatrix();
      }
   }
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
   draw_rigids();
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
