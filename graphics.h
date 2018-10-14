#/* Graphics header file */

#define GRAPHICS

#include <GL/gl.h>
#include <GL/glu.h>
/* #include <GL/glut.h> */
/* #include <GL/glx.h> */

/* Standard I/O header */
#include <stdio.h>
#include <stdlib.h>

/* ODE */
#include <ode/ode.h>

/* Mathematics header used for geometry */
#include <math.h>

#define X_FROM_RY(ry, x, z) (cos(DEG_TO_RAD(ry))*x) - (sin(DEG_TO_RAD(ry))*z)
#define Z_FROM_RY(ry, x, z) (sin(DEG_TO_RAD(ry))*x) + (cos(DEG_TO_RAD(ry))*z)

/* Basic container for a graphic */
struct Image {
  unsigned long width;
  unsigned long height;
  unsigned short int bpp;
  char *data;
};

/* An image */
typedef struct Image Image;

/* Collison detection list for the road spline object or
   any other triangle meshes */
struct TriColData {
  dVector3 *vertex;
  int *index;


  struct TriColData *next;
};

struct TriColData *first;
struct TriColData *current;

/* Basic container for an object.
   An object consists of x,y,z values, rotations, and
   a display list which is associated with that object */
struct Object3D {
  float x;
  float y;
  float z;

  /* Rotation matrix */
  float r[12];

  /* Pointer to the display function of this object */
  void *display_function;

  /* Pointer to the physics function of this object */
  void *physics_function;

  /* Pointer to a set of physics data held about the object */
  void *physics_data;

  /* The texture this object uses */
  int texture;

  int display_list;

  /* Do we translate or rotate the object prior to drawing? */
  int no_translate;
};

/* Basic container for a camera.
   A camera is basically a reference to the object it's attached to,
   plus offset values for x,y and z and relevant rotations */
struct Camera3D {
  GLfloat x;
  GLfloat y;
  GLfloat z;

  /* Rotation matrix */
  float r[12];

  /* Whether to track various things */
  unsigned int track_x;
  unsigned int track_y;
  unsigned int track_z;
  unsigned int track_r;

  unsigned int track_mode;

  /* Is the camera for a car interior? */
  int interior;

  int object;

};


struct Light3D {
  float position[4];
  float ambient[4];
  float diffuse[4];
  float attenuation[3];

  int flicker;
  long last_flickered;

  float flicker_colour[4];

  int follow_car;
  /* Physics data if we follow a car */
  void *car;

  /* position on car */
  float pos[3];
};

struct Artifact {
  /* Current position */
  float x;
  float y;
  float z;

  /* Current speed */
  float dx;
  float dy;
  float dz;

  /* Acceleration (constant) */
  float ax;
  float ay;
  float az;

  /* Life and activity */
  float life;
  float decay;
  int active;

  /* Size and growth rate */
  float size;
  float growth;

  /* Colour (woo yay!) */
  float r;
  float g;
  float b;

  /* Is it affected by lighting? */
  int lit;
};
