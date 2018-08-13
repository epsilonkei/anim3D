#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "floors_fluid.hpp"
#include <sys/stat.h>

#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;
extern double PI;

static GLubyte *pixels = NULL;
static const GLenum FORMAT = GL_RGBA;
static const GLuint FORMAT_NBYTES = 4;
static unsigned int nscreenshots = 0;
static double timeInSim = 0.0;

static int count = 0;

// Global variables
char img_folder[] = "/tmp/water/";
char title[] = "Full-Screen & Windowed Mode";  // Windowed mode's title
int windowWidth  = 640;     // Windowed mode's width
int windowHeight = 480;     // Windowed mode's height
int windowPosX   = 50;      // Windowed mode's top-left corner x
int windowPosY   = 50;      // Windowed mode's top-left corner y

int refreshMillis = 30;      // Refresh period in milliseconds
double dt = refreshMillis * 1e-3;
// double dt = 1e-3;
int frameRate = 1000 / refreshMillis;
#define MAX_TIME 30

#define N_part_per_fluid 500
#define N_fluid 1
double part_mass = 1e-3, part_radius = 0.05;
double prev_poss[N_part_per_fluid * N_fluid][3], poss[N_part_per_fluid * N_fluid][3],
   vels[N_part_per_fluid * N_fluid][3], accs[N_part_per_fluid * N_fluid][3];

#define N_floor 6
double floor_elass[N_floor];
double floor_orgs[N_floor][3], floor_norms[N_floor][3];

#define cup_side 1.0
#define h_cup 4.0
#define slide_angle 30.0 * PI / 180
floors_fluid FF;

static double org_dist = 13.0, org_pitch = 70.0, org_yaw = 0.0;
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
   glReadBuffer(GL_BACK);
   // glClearColor (1.0, 1.0, 1.0, 1.0);
   glClearColor (0.0, 0.0, 0.0, 1.0);
   glClearDepth( 1.0 );
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glViewport(0, 0, windowWidth, windowHeight);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glMatrixMode(GL_MODELVIEW);
   pixels = (GLubyte*)malloc(FORMAT_NBYTES * windowWidth * windowHeight);

   struct stat sb;
   if (!stat(img_folder, &sb) == 0 || !S_ISDIR(sb.st_mode)) {
      const int dir_err = mkdir(img_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (dir_err == -1) {
         std::cerr <<"Error creating directory!" << std::endl;
         exit(1);
      }
   }

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

void deinit()  {
    free(pixels);
}

static void create_ppm(char *prefix, int frame_id, unsigned int width, unsigned int height,
        unsigned int color_max, unsigned int pixel_nbytes, GLubyte *pixels) {
    size_t i, j, k, cur;
    enum Constants { max_filename = 256 };
    char filename[max_filename];
    snprintf(filename, max_filename, "%s%d.ppm", prefix, frame_id);
    FILE *f = fopen(filename, "w");
    fprintf(f, "P3\n%d %d\n%d\n", width, windowHeight, 255);
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            cur = pixel_nbytes * ((height - i - 1) * width + j);
            fprintf(f, "%3d %3d %3d ", pixels[cur], pixels[cur + 1], pixels[cur + 2]);
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

void initFloor() {
   floor_elass[0] = 0.8;
   floor_orgs[0][0] = floor_orgs[0][1] = floor_orgs[0][2] = 0;
   floor_norms[0][0] = floor_norms[0][1] = 0; floor_norms[0][2] = 1;
   FF.add_floor(floor_orgs[0], floor_norms[0], floor_elass[0]);
}

void initCup() {
   // for side wall
   floor_elass[1] = 0.8;
   floor_orgs[1][0] = -cup_side;floor_orgs[1][1] = 0;floor_orgs[1][2] = 0;
   floor_norms[1][0] = 1;floor_norms[1][1] = 0; floor_norms[1][2] = 0;
   FF.add_floor(floor_orgs[1], floor_norms[1], floor_elass[1]);
   FF.floors[1]->set_ylimit(-cup_side, cup_side);
   FF.floors[1]->set_zlimit(0, h_cup);
   //
   floor_elass[2] = 0.8;
   floor_orgs[2][0] = cup_side;floor_orgs[2][1] = 0;floor_orgs[2][2] = 0;
   floor_norms[2][0] = -1;floor_norms[2][1] = 0; floor_norms[2][2] = 0;
   FF.add_floor(floor_orgs[2], floor_norms[2], floor_elass[2]);
   double z_min_slide = 0.2 * h_cup;
   FF.floors[2]->set_ylimit(-cup_side, cup_side);
   FF.floors[2]->set_zlimit(0, z_min_slide);
   //
   floor_elass[3] = 0.8;
   floor_orgs[3][0] = 0;floor_orgs[3][1] = -cup_side;floor_orgs[3][2] = 0;
   floor_norms[3][0] = 0;floor_norms[3][1] = 1; floor_norms[3][2] = 0;
   FF.add_floor(floor_orgs[3], floor_norms[3], floor_elass[3]);
   FF.floors[3]->set_xlimit(-cup_side, cup_side);
   FF.floors[3]->set_zlimit(0, h_cup);
   //
   floor_elass[4] = 0.8;
   floor_orgs[4][0] = 0;floor_orgs[4][1] = cup_side;floor_orgs[4][2] = 0;
   floor_norms[4][0] = 0;floor_norms[4][1] = -1; floor_norms[4][2] = 0;
   FF.add_floor(floor_orgs[4], floor_norms[4], floor_elass[4]);
   FF.floors[4]->set_xlimit(-cup_side, cup_side);
   FF.floors[4]->set_zlimit(0, h_cup);
}

void initSlide() {
   double z_min_slide = 0.2 * h_cup;
   floor_elass[5] = 0.8;
   floor_orgs[5][0] = cup_side;floor_orgs[5][1] = cup_side;floor_orgs[5][2] = z_min_slide;
   floor_norms[5][0] = -sin(slide_angle);floor_norms[5][1] = 0;floor_norms[5][2] = cos(slide_angle);
   FF.add_floor(floor_orgs[5], floor_norms[5], floor_elass[5]);
   //
   double x_max_slide = 5 * cup_side;
   double z_max_slide = (- (x_max_slide - cup_side) * FF.floors[5]->norm_vec[0])
      / FF.floors[5]->norm_vec[2] + z_min_slide;
   FF.floors[5]->set_xlimit(cup_side, x_max_slide);
   FF.floors[5]->set_ylimit(-cup_side, cup_side);
   FF.floors[5]->set_zlimit(z_min_slide, z_max_slide);
}

void initFluid() {
   for (uint i = 0; i < N_fluid; i++) {
      for (uint j = 0; j < N_part_per_fluid; j++) {
         poss[i*8+j][0] = (double(rand()) / RAND_MAX * 2 * cup_side + 3 * cup_side ) * 0.95;
         poss[i*8+j][1] = (double(rand()) / RAND_MAX * 2 * cup_side - cup_side) * 0.95;
         poss[i*8+j][2] = double(rand()) / RAND_MAX * h_cup + 1.5*h_cup;
         //
         prev_poss[i*8+j][0] = poss[i*8+j][0];
         prev_poss[i*8+j][1] = poss[i*8+j][1];
         prev_poss[i*8+j][2] = poss[i*8+j][2];
         vels[i*8+j][0] = 0; vels[i*8+j][1] = 0; vels[i*8+j][2] = 0;
         accs[i*8+j][0] = 0; accs[i*8+j][1] = 0; accs[i*8+j][2] = 0;
         FF.add_particle(part_mass, part_radius, dt, prev_poss[i*8+j], poss[i*8+j],
                         vels[i*8+j], accs[i*8+j], i, 0.3, 20.0);
      }
   }
   for (uint i = 0; i < FF.fluids.size(); i++) {
      FF.fluids[i]->init();
      for (uint j = 0; j < FF.fluids[i]->pl.size(); j++) {
         FF.fluids[i]->pl[i]->init();
         FF.fluids[i]->pl[i]->dens = FF.fluids[i]->init_dens;
      }
   }
}

void initSim() {
   initFluid();
   initFloor();
   initCup();
   initSlide();
}

void physics_calculate(){
   FF.update_movement(dt);
   timeInSim += dt;
   count ++;
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

void draw_cup(){
   glDisable(GL_LIGHTING); glBegin(GL_QUADS);
   glColor3f(0.5, 0.5, 0.5);
   glVertex3f ( cup_side, -cup_side, 0);
   glVertex3f ( cup_side,  cup_side, 0);
   glVertex3f (-cup_side,  cup_side, 0);
   glVertex3f (-cup_side, -cup_side, 0);
   //
   glColor3f(0.2, 0.2, 0.2);
   double z_min_slide = FF.floors[5]->min_z;
   glVertex3f ( cup_side, -cup_side, 0);
   glVertex3f ( cup_side,  cup_side, 0);
   glVertex3f ( cup_side,  cup_side, z_min_slide);
   glVertex3f ( cup_side, -cup_side, z_min_slide);
   //
   glVertex3f ( cup_side,  cup_side, 0);
   glVertex3f (-cup_side,  cup_side, 0);
   glVertex3f (-cup_side,  cup_side, h_cup);
   glVertex3f ( cup_side,  cup_side, h_cup);
   //
   glVertex3f (-cup_side,  cup_side, 0);
   glVertex3f (-cup_side, -cup_side, 0);
   glVertex3f (-cup_side, -cup_side, h_cup);
   glVertex3f (-cup_side,  cup_side, h_cup);
   //
   // glVertex3f (-cup_side, -cup_side, 0);
   // glVertex3f ( cup_side, -cup_side, 0);
   // glVertex3f ( cup_side, -cup_side, h_cup);
   // glVertex3f (-cup_side, -cup_side, h_cup);
   //
   glEnd(); glEnable(GL_LIGHTING);
}

void draw_slide() {
   glDisable(GL_LIGHTING); glBegin(GL_QUADS);
   glColor3f(0.2, 0.2, 0.2);
   double x_max_slide = FF.floors[5]->max_x;
   double z_min_slide = FF.floors[5]->min_z;
   double z_max_slide = FF.floors[5]->max_z;
   // slide
   glVertex3f ( cup_side, -cup_side, z_min_slide);
   glVertex3f ( cup_side,  cup_side, z_min_slide);
   glVertex3f ( x_max_slide,  cup_side, z_max_slide);
   glVertex3f ( x_max_slide, -cup_side, z_max_slide);
   //
   glEnd(); glEnable(GL_LIGHTING);
}

void draw_fluid() {
   for (int i = 0; i<FF.fluids.size(); i++) {
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color[(i + 2) % (sizeof(color)/sizeof(*color))]);
      for (int j = 0; j < FF.fluids[i]->pl.size(); j++) {
         glPushMatrix();
         glTranslatef(FF.fluids[i]->pl[j]->pos[0], FF.fluids[i]->pl[j]->pos[1],
                      FF.fluids[i]->pl[j]->pos[2]);
         glutSolidSphere (FF.fluids[i]->pl[j]->radius, 16, 16);
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
#if ENABLE_TIMER
   stop_watch display_timer = stop_watch();
   display_timer.start();
#endif
   draw_floor();
   draw_origin();
   draw_cup();
   draw_slide();
   draw_fluid();
   glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
#if ENABLE_TIMER
   display_timer.stop();
   // std::cerr << "display_time: " << display_timer.getTime() << std::endl;
   std::cerr << display_timer.getTime() << std::endl;
#endif
#if ENABLE_SCREEN_SHOT
   glReadPixels(0, 0, windowWidth, windowHeight, FORMAT, GL_UNSIGNED_BYTE, pixels);
   puts("screenshot");
   // create_ppm("/tmp/water/", nscreenshots, windowWidth, windowHeight, 255, FORMAT_NBYTES, pixels);
   create_ppm(img_folder, nscreenshots, windowWidth, windowHeight, 255, FORMAT_NBYTES, pixels);
   nscreenshots++;
#endif //ENABLE_SCREEN_SHOT
   if (timeInSim > MAX_TIME) exit(0);
   // if (count >= 5) exit(0);
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
   atexit(deinit);
   glutMainLoop();               // Enter event-processing loop
   return 0;
}
