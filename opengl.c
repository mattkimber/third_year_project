/*
 * Third Year Project
 * Matt Kimber
 *
 * This is the main code file for the project - it handles all initialisation,
 * loading and drawing functions in the code.
 */

/* Configuration */
#ifndef CONFIG
#include "config.h"
#endif

#define SPLINESTEPS 4

/* Standard C string library - strlen() and others */
#include <string.h>

/* Character library used for toupper() */
#include <ctype.h>

/* SDL is used for keyboard input */
#include <SDL/SDL.h>

/* Standard rock stars */
#define BrianMay

/* Font */
#define STD_FONT "-adobe-helvetica-medium-r-normal-*-18-*-*-*-p-*-iso8859-1"

/* Our own local header files */
#ifndef GRAPHICS
#include "graphics.h"
#endif

#ifndef PHYSICS
#include "physics.h"
#endif

/* Maximum number of artifacts */
#define MAX_ARTIFACTS 5000

/* Used to hold details about the window */
int bpp = 0, flags = 0;

/* Window handle */
int window;

/* How many objects are in the scene */
int scn_objects;

/* Texture map storage */
unsigned int *texture;

/* The current wheel we are loading */
int current_wheel = 0;

/* Cameras */
int cameras;
int cur_cam;
float camera_ry = 0.0;

/* Base display list for the text */
GLuint font_base;

/* An object */
struct Object3D *obj;

/* A camera */
struct Camera3D *camera;

/* The lighting */
struct Light3D *light;
int lights;

/* Artifacts */
struct Artifact artifact[MAX_ARTIFACTS];
int cur_artifact = 0;
int artifact_texture;

int car_int_texture;

/* Physics functions */
void PHNothing(struct Object3D *);
void PHGeneric(struct Object3D *);
void PHGauge(struct Object3D *);
void PHCar(struct Object3D *);
void PHTrailer2Wheel(struct Object3D *);
void PHWheel(struct Object3D *);
void PHArtifactSpawn(struct Object3D *);

void PHFollow(struct Object3D *);

void PHLightFlicker(struct Light3D *);
void PHLightCarMount(struct Light3D *);

void PHStepWorld();
void PHPreStepWorld();
int PHShouldDrawGraphics();


void UpdateAxes();
void UpdateTimingInfo();

/* Sound functions */
void Snd_LoadSound(char *, int);
void Snd_FreeSounds();
void Snd_CloseAudio();
void Snd_StartSampleLoop(int, int);
void Snd_PlaySound(int, int);
void Snd_SetVolume(int, int);
void Snd_StartAudio();
void Snd_AddEffect(int, int);
void Snd_SetPitch(float);
void Snd_LoadMP3(char *);
void Snd_PlayPauseMusic();
void Snd_MusicVolumeUp();
void Snd_MusicVolumeDown();

void SetValveOpenPos(int, float);
void SetValveOpenDuration(int, float);
void SetValvePressureDrop(int, float);
void SetCylOffset(int, float);
void SetCylinders(int);

/* Matrix functions */
void MatrixTranspose(dMatrix3, dMatrix3);
void MatrixMultiply(dMatrix3, dMatrix3, dMatrix3);
void GenerateRotationMatrix(dMatrix3, float, float, float);

/* Bitmap functions */
int LoadBitmap(const char *, Image *);
int LoadTGA(const char *, Image *);



/* Close the current game scene */
void CloseScene()
{
  /* Destroy the world of game objects */
  dWorldDestroy(game_world);

  /* Free the sounds */
  Snd_FreeSounds();

  /* Free the object, texture and camera arrays */
  free(texture);
  free(obj);
  free(camera);
  free(tyre_col);
  current_wheel = 0;

}

/* Quit the program safely */
void Quit()
{

  /* Exit */
  CloseScene();

  dCloseODE();

  Snd_CloseAudio();

  SDL_QuitSubSystem(SDL_INIT_AUDIO);
  SDL_QuitSubSystem(SDL_INIT_JOYSTICK);

  SDL_Quit();

  exit(0);
}


/*
 * OpenGL Initialisation function - generic.  Sets all parameters
 * required for OpenGL
 *
 * Called after OpenGL window is created
 */

void InitGL(const int width, const int height)
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); /* Clear BG to black */

  glClearDepth(1.0); /* Clear the Z-buffer */

  glDepthFunc(GL_LESS); /* Type of Z-test */
  glEnable(GL_DEPTH_TEST); /* Enable Z-buffering */
  glShadeModel(GL_SMOOTH); /* Enable smooth colour shading */

  #ifdef USEALPHA
  glEnable(GL_ALPHA_TEST); /* Enable alpha testing (transparency) */
  glAlphaFunc(GL_EQUAL, 1.0);
  #endif



  glMatrixMode(GL_PROJECTION);
  glLoadIdentity(); /* Reset the projection matrix */

  /* Calculate aspect ratio */
  gluPerspective(45.0,(GLfloat)width/(GLfloat)height,0.1f,100.0f);

  glMatrixMode(GL_MODELVIEW);

}


/* Called on window resize */
void ResizeGLScene(const int width, int height)
{
  if(height == 0) height = 1; /* Protection against tiny window */

  /* Reset viewport and projection */
  glViewport(0,0,width,height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
}

/* Function used to bind textures into OpenGL */
void LoadGLTexture(const char *filename, const int texture_num) {
  #ifdef TEXTURE

  /* Load the texture */
  Image *img;
  GLuint type = GL_RGB;

  /* Allocate space for the texture to be stored */
  img = (Image *) malloc(sizeof(Image));

  if (img == NULL) {
    printf("Could not allocate memory to load texture.");
    /* Program has failed! This should probably be treated better. */
    exit(23);
  }

  /* Load tga or bmp file */
  if (strcmp(".tga", &filename[strlen(filename) - 4]) == 0) {
    /* Call the Targa loader */
    if (!LoadTGA(filename, img)) exit(23);
  } else {
    /* Call the bitmap loader */
    if (!LoadBitmap(filename, img)) exit(23);
  }

  /* Create an OpenGL texture */
  glGenTextures(1, &texture[texture_num]);

  /* A 2D texture */
  glBindTexture(GL_TEXTURE_2D, texture[texture_num]);

  /* Set the texture scaling type - linear for best quality */
#ifdef NICETEXTURES
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#else
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
#endif


  /* If the image is 32 bits, there is an alpha channel */
  if (img->bpp == 32) {
    #ifndef NODEBUG
    printf("Found an image with alpha channel.\n");
    #endif
    type = GL_RGBA;
  }

  /* Set the texture - a 2D texture with LOD 0, RGB, x size and y size */
#ifdef NICETEXTURES
  if(type == GL_RGBA) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, type, img->width, img->height, 0,
		 type, GL_UNSIGNED_BYTE, img->data);
  } else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		    GL_LINEAR_MIPMAP_LINEAR);

    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, img->width, img->height,
		      type, GL_UNSIGNED_BYTE, img->data);
  }
#else
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0, type, img->width, img->height, 0,
		 type, GL_UNSIGNED_BYTE, img->data);
#endif

#endif

}


/* Printing function for text */
GLvoid GLPrint(const char *txt)
{
  /* If there's no text, then return */
  if (txt == NULL) {
    return;
  }

  /* Alert OpenGL that we're going to offset the display lists */
  glPushAttrib(GL_LIST_BIT);

  /* Set the base character to 32 */
  glListBase(font_base - 32);

  /* Draws the display list for the text */
  glCallLists(strlen(txt), GL_UNSIGNED_BYTE, txt);

  /* Undo the glPushAttrib we did earlier */
  glPopAttrib();

}

void GFXSetTransform(const float x, const float y, const float z,
		     const float r[12])
{
  GLfloat m[16];

  /* Fill out an OpenGL-style matrix for rotation and translation */
  m[0] = r[0]; m[1] = r[4]; m[2] = r[8]; m[3] = 0.0;
  m[4] = r[1]; m[5] = r[5]; m[6] = r[9]; m[7] = 0.0;
  m[8] = r[2]; m[9] = r[6]; m[10] = r[10]; m[11] = 0.0;
  m[12] = x; m[13] = y; m[14] = z; m[15] = 1.0;

  /* Put the matrix on the stack */
  glPushMatrix();
  glMultMatrixf(m);

}

void SetCameraTransform()
{
  float x=0.0, y=0.0, z=0.0;
  dMatrix3 r;
  dMatrix3 v;
  dMatrix3 M1, M2;
  int j;


  if(camera[cur_cam].track_r) {
    MatrixTranspose(obj[camera[cur_cam].object].r, M1);
  } else {
    for(j=0;j<12;j++) M1[j] = camera[cur_cam].r[j];
  }

  dRFromAxisAndAngle(M2, 0, 1, 0, camera_ry);


  dRSetIdentity(v);

  if(camera[cur_cam].track_x) x = obj[camera[cur_cam].object].x;
  if(camera[cur_cam].track_y) y = obj[camera[cur_cam].object].y;
  if(camera[cur_cam].track_z) z = obj[camera[cur_cam].object].z;

  if (camera[cur_cam].track_mode == 1) {
    glTranslatef(0.0,-1.0,-5.0);

    MatrixMultiply(M1, M2, r);
    GFXSetTransform(-camera[cur_cam].x, -camera[cur_cam].y,
		    -camera[cur_cam].z, r);
  } else {

    GFXSetTransform(0,0,0, M2);

    glTranslatef(0.0,-1.0,-5.0);

    GFXSetTransform(-camera[cur_cam].x, -camera[cur_cam].y,
		    -camera[cur_cam].z, M1);

  }

  GFXSetTransform(-x, -y, -z, v);

}


float rnd_f(float f)
{
  return (((float)(rand() % 10000) / 10000.0) - 0.5) * f;
}

void GFXSpawnArtifacts(const struct ArtifactSpawnProperties *prop, int num)
{
  int i,j;
  static int timeout = 0;

  j = 0;

  if (timeout > 0) {
    timeout--;
    return;
  }

  /* Spawn some new artifacts */
  for(i=0;i<num;i++) {

    /* Only add an artifact if there is a spare slot for one */
    while(artifact[cur_artifact].active) {
      cur_artifact = (cur_artifact + 1) % (MAX_ARTIFACTS);
      j++;
      if(j>MAX_ARTIFACTS) {
	/* Don't keep trying to add artifacts - wait for them to decay */
	timeout = 50;
	return;
      }
    }

    artifact[cur_artifact].life = prop->life;
    artifact[cur_artifact].active = 1;
    artifact[cur_artifact].decay = prop->decay + rnd_f(prop->vdecay);

    artifact[cur_artifact].x = prop->x;
    artifact[cur_artifact].y = prop->y;
    artifact[cur_artifact].z = prop->z;

    artifact[cur_artifact].dx = prop->dx + rnd_f(prop->vdx);
    artifact[cur_artifact].dy = prop->dy + rnd_f(prop->vdy);
    artifact[cur_artifact].dz = prop->dz + rnd_f(prop->vdz);

    artifact[cur_artifact].ax = prop->ax;
    artifact[cur_artifact].ay = prop->ay;
    artifact[cur_artifact].az = prop->az;

    artifact[cur_artifact].r = prop->r + rnd_f(prop->vr);
    artifact[cur_artifact].g = prop->g + rnd_f(prop->vg);
    artifact[cur_artifact].b = prop->b + rnd_f(prop->vb);

    artifact[cur_artifact].size = prop->size;
    artifact[cur_artifact].growth = prop->growth + rnd_f(prop->vgrowth);

    artifact[cur_artifact].lit = prop->lit;
  }

}

void InitArtifacts()
{
  int i;

  /* Initialise all the artifacts */
  for(i=0;i<MAX_ARTIFACTS;i++) {
    artifact[i].active = 0;
  }
}

/* Draw the artifacts (after drawing the scene) */
void GFXArtifacts(const long frames)
{
  int i, j;
  float x, y, z, s[5][3];
  float v[5][3] = {{1.0, 1.0, 0.0},{-1.0,1.0,0.0},
		   {1.0,-1.0,0.0},{-1.0,-1.0,0.0},
                   {0.0, 0.0, 1.0} };

  float M[16];
  float scale;

  scale = ((float)frames / 3.0);

  glLoadIdentity();

  dRSetIdentity(M);

  /* Change the alpha function to a blending one in order to do the
     artifact effects. */
  #ifdef USEALPHA
  glEnable(GL_BLEND);
  glDisable(GL_ALPHA_TEST);


  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE); /* Disable Z-buffer writes */
  #endif


  SetCameraTransform();

  /* Get the matrix */
  glGetFloatv(GL_MODELVIEW_MATRIX, M);

#ifdef TEXTURE
  glBindTexture(GL_TEXTURE_2D, texture[artifact_texture]);
#endif

  for(i=0;i<MAX_ARTIFACTS;i++) {

    if(artifact[i].active) {

#ifdef LIGHTING
      if (artifact[i].lit) {
	glEnable(GL_LIGHTING);
      } else {
	glDisable(GL_LIGHTING);
      }
#endif

      /* Update positions */
      artifact[i].dx += scale * artifact[i].ax;
      artifact[i].dy += scale * artifact[i].ay;
      artifact[i].dz += scale * artifact[i].az;

      artifact[i].x += scale * artifact[i].dx;
      artifact[i].y += scale * artifact[i].dy;
      artifact[i].z += scale * artifact[i].dz;

      artifact[i].size += scale * artifact[i].growth;

      x = artifact[i].x;
      y = artifact[i].y;
      z = artifact[i].z;

      for(j=0;j<5;j++) {
	s[j][0] = (M[0] * (artifact[i].size * v[j][0])) +
	          (M[1] * (artifact[i].size * v[j][1])) +
	          (M[2] * (artifact[i].size * v[j][2]));
	s[j][1] = (M[4] * (artifact[i].size * v[j][0])) +
	          (M[5] * (artifact[i].size * v[j][1])) +
	          (M[6] * (artifact[i].size * v[j][2]));
	s[j][2] = (M[8] * (artifact[i].size * v[j][0])) +
	          (M[9] * (artifact[i].size * v[j][1])) +
	          (M[10] * (artifact[i].size * v[j][2]));
      }

      /* Decay the artifact */
      artifact[i].life -= scale * artifact[i].decay;
      if(artifact[i].life <= 0.0) {
	artifact[i].active = 0;
      }

      /* Draw this artifact */
      /* Set the color (including alpha value) */
      glColor4f(artifact[i].r,
		artifact[i].g,
		artifact[i].b,
		artifact[i].life);


      /* Draw it from a triangle strip for maximal speed */
      glBegin(GL_TRIANGLE_STRIP);

      glNormal3f(s[4][0], s[4][1], s[4][2]);
      glTexCoord2d(1,1); glVertex3f(x+s[0][0], y+s[0][1], z+s[0][2]);
      glTexCoord2d(0,1); glVertex3f(x+s[1][0], y+s[1][1], z+s[1][2]);
      glTexCoord2d(1,0); glVertex3f(x+s[2][0], y+s[2][1], z+s[2][2]);
      glTexCoord2d(0,0); glVertex3f(x+s[3][0], y+s[3][1], z+s[3][2]);
      glNormal3d(0.0, 0.0, 1.0);

      glEnd();


    }
  }



  #ifdef USEALPHA
  glDisable(GL_BLEND);

#ifdef LIGHTING
  glEnable(GL_LIGHTING);
#endif

  glEnable(GL_ALPHA_TEST);
  glDepthMask(GL_TRUE); /* Enable Z-buffer writes */
  #endif

}



/* 3D object drawing function */
void GFXDisplayList(const int i)
{
  glLoadIdentity(); /* Reset the view */

  /* If the object is not to be translated, then we don't do so */
  if (!obj[i].no_translate) {
    SetCameraTransform();
  } else {
    glTranslatef(0.0,-1.0,-5.0);
#ifdef LIGHTING
    glDisable(GL_LIGHTING);
#endif
  }


  GFXSetTransform(obj[i].x, obj[i].y, obj[i].z, obj[i].r);

  /* If the object has a texture, bind it */
#ifdef TEXTURE
  if (obj[i].texture == 0) {
    glBindTexture(GL_TEXTURE_2D, 0);
  } else {
    glBindTexture(GL_TEXTURE_2D, texture[obj[i].texture - 1]);
  }
#endif

  /* Draw the display list */
  glCallList(obj[i].display_list);

  /* Remove the pushed matrix */
  glPopMatrix();  glPopMatrix(); glPopMatrix();

#ifdef LIGHTING
  glEnable(GL_LIGHTING);
#endif
}


/* 3D object drawing function */
void GFXCarWheel(const int i)
{
  /* If we are inside a car, don't draw the wheel! */
  if(camera[cur_cam].interior) return;

  glLoadIdentity(); /* Reset the view */

  /* Wheels are always translated */
  SetCameraTransform();

  GFXSetTransform(obj[i].x, obj[i].y, obj[i].z, obj[i].r);

  /* If the object has a texture, bind it */
#ifdef TEXTURE
  if (obj[i].texture == 0) {
    glBindTexture(GL_TEXTURE_2D, 0);
  } else {
    glBindTexture(GL_TEXTURE_2D, texture[obj[i].texture - 1]);
  }
#endif

  /* Draw the display list */
  glCallList(obj[i].display_list);

  /* Remove the pushed matrix */
  glPopMatrix();  glPopMatrix(); glPopMatrix();

#ifdef LIGHTING
  glEnable(GL_LIGHTING);
#endif
}


/* 3D object drawing function */
void GFXCar(const int i)
{
  glLoadIdentity(); /* Reset the view */

  /* Cars are always translated */
  SetCameraTransform();

  GFXSetTransform(obj[i].x, obj[i].y, obj[i].z, obj[i].r);

  /* If the object has a texture, bind it.  If we are in the car interior,
     instead bind the interior texture */
#ifdef TEXTURE
  if (obj[i].texture == 0) {
    glBindTexture(GL_TEXTURE_2D, 0);
  } else {
    if(camera[cur_cam].interior) {
      glBindTexture(GL_TEXTURE_2D, texture[car_int_texture]);
    } else {
      glBindTexture(GL_TEXTURE_2D, texture[obj[i].texture - 1]);
    }

  }
#endif

  /* Draw the display list */
  glCallList(obj[i].display_list);

  /* Remove the pushed matrix */
  glPopMatrix();  glPopMatrix(); glPopMatrix();

#ifdef LIGHTING
  glEnable(GL_LIGHTING);
#endif
}

/* Null graphics routine */
void GFXNone(const int i)
{

}

/* Light update function */
void GFXUpdateLight(const int i)
{
  if (light[i].flicker > 0) {
    PHLightFlicker(&light[i]);
    glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, light[i].diffuse);
  }

  if (light[i].follow_car) {
    PHLightCarMount(&light[i]);
  }

  glLightfv(GL_LIGHT0 + i, GL_POSITION, light[i].position);

}

/* Text routines */
void GFXText(const int i)
{
  #ifdef LIGHTING
  glDisable(GL_LIGHTING);
  #endif

  glLoadIdentity(); /* Reset the view */

  /* Camera Translate */
  glTranslatef(-0.025,0.035,-0.1);

  /* Rend0r some text-like textesque thing */
  glColor3f(0.8, 0.8, 0.8);
  glRasterPos2f(0.0, 0.0);

  GLPrint(score_string);

  #ifdef LIGHTING
  glEnable(GL_LIGHTING);
  #endif
}

/* Main drawing function */
void DrawGLScene()
{

  int i;
  void (*fptr)(struct Object3D *);
  void (*dfptr)(int);

  /* Do we draw the current frame, or only simulate it? */
  int draw_frame;
  static long physics_frames = 0;

  draw_frame = PHShouldDrawGraphics();


  if (draw_frame) {

    /* Clear screen and Z-Buffer */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#ifdef LIGHTING
    glEnable(GL_LIGHTING);

    glLoadIdentity();
    SetCameraTransform();

    for (i=0;i<lights;i++) {
      GFXUpdateLight(i);
    }
#endif


  } else {
    /* Step the ODE world first */
    PHPreStepWorld();
  }


  /* Render all objects in scene */
  for (i=0;i<scn_objects;i++) {

    if (draw_frame) {
      dfptr = (void *)obj[i].display_function;
      dfptr(i);
    } else {
      fptr = (void *)obj[i].physics_function;
      fptr(&obj[i]);
    }

  }

  if (draw_frame) {

#ifdef TEXT
    GFXText(0, rx, ry, rz, x, y, z);
#endif

#ifdef ARTIFACTS
    GFXArtifacts(physics_frames);
#endif

#ifdef LIGHTING
    glDisable(GL_LIGHTING);
#endif

    /* Swap primary and back buffer */
    SDL_GL_SwapBuffers();

    /* Sound */
    Snd_SetPitch(car_rpm);

    physics_frames = 0;

  } else {
    physics_frames++;

    PHStepWorld();

    UpdateTimingInfo();
  }

  /* Axis returns */
  UpdateAxes();



}


GLuint ReadObject(const char *filename)
{
  /* Reads the object specified by filename and builds the display list
     pointed to by display_list */

  /* File for input */
  FILE *file;

  int file_version;

  /* Type of line */
  char instruction;
  /* Parameters */
  float f1, f2, f3;

  /* Pointer to the display list */
  GLuint display_list;


  #ifndef NODEBUG
  printf("Generating list\n");

  /* Try and open the object file */
  printf("Opening object file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /*
   * Validate the file.  Currently this consists of
   * checking it's a version 1.
   */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
    return 0;
  }

  /* Allocate memory for the display list */
  display_list=glGenLists(1);

  /* Begin the display list */
  glNewList(display_list,GL_COMPILE);

  #ifndef NODEBUG
  printf("Begin list.\n");
  #endif

  /* Create the list */

  /* Find out whether we're doing triangles or quads */
  fscanf(file, "%c\n", &instruction);

  if(toupper(instruction) == 'T') {
    glBegin(GL_TRIANGLES);
  } else if (toupper(instruction) == 'Q') {
    glBegin(GL_QUADS);
  }

  while(!feof(file)) {

    /* Input the next line of the file */
    fscanf(file, "%c %f %f %f\n", &instruction, &f1, &f2, &f3);

    if(toupper(instruction) == 'V') {
      /* Vertex */
      glVertex3f(f1, f2, f3);
    } else if (toupper(instruction) == 'C') {
      /* Colour */
      glColor3f(f1, f2, f3);
    } else if (toupper(instruction) == 'N') {
      /* Normal */
      glNormal3f(f1, f2, f3);
    } else if (toupper(instruction) == 'T') {
      glTexCoord2f(f1, f2);
    }

  }

  /* End list creation */
  glEnd();

  #ifndef NODEBUG
  printf("End list.\n");
  #endif

  /* End the list */
  glEndList();

  #ifndef NODEBUG
  printf("List: %d.\n", display_list);
  #endif

  /* Close the input file */
  #ifndef NODEBUG
  printf("Closing object file.\n");
  #endif

  fclose(file);

  /* Return the display list */
  return display_list;

}

/* Cross product functions */
float CrossX(float v1[3], float v2[3])
{
  return (v1[1] * v2[2]) - (v1[3] * v2[1]);
}

float CrossY(float v1[3], float v2[3])
{
  return (v1[2] * v2[0]) - (v1[0] * v2[2]);
}

float CrossZ(float v1[3], float v2[3])
{
  return (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

void GenPolygon(const float f1, const float f2, const float f3,
		const float f4, const float f5, const float f6)
{
  dTriMeshDataID mesh;
  dGeomID tri_mesh;

  const float road_width = 3.5;

  int i;
  int *index;
  dVector3 *vertex;
  float normal[3], tv1[3], tv2[3], mag;

  current->index = malloc(6 * sizeof(int));
  current->vertex = malloc(4 * sizeof(dVector3));

  vertex = current->vertex;
  index = current->index;

  /* Collision detection geometry */
  vertex[0][0] = f1 - road_width;  vertex[0][1] = f2;  vertex[0][2] = f3;
  vertex[1][0] = f1 + road_width;  vertex[1][1] = f2;  vertex[1][2] = f3;
  vertex[2][0] = f4 - road_width;  vertex[2][1] = f5;  vertex[2][2] = f6;
  vertex[3][0] = f4 + road_width;  vertex[3][1] = f5;  vertex[3][2] = f6;

  index[0] = 3; index[1] = 2;  index[2] = 1;
  index[3] = 0; index[4] = 1;  index[5] = 2;

  /* Compute the vertex normal for the polygon */
  for(i=0;i<3;i++) {
    tv1[i] = vertex[1][i] - vertex[0][i];
    tv2[i] = vertex[2][i] - vertex[1][i];
  }

  normal[0] = CrossX(tv1, tv2);
  normal[1] = CrossY(tv1, tv2);
  normal[2] = CrossZ(tv1, tv2);

  mag = sqrt((normal[0] * normal[0]) + (normal[1] * normal[1]) +
	     (normal[2] * normal[2]));

  normal[0] = normal[0] / mag;
  normal[1] = normal[1] / mag;
  normal[2] = normal[2] / mag;


  mesh = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSimple(mesh, (const dVector3 *)vertex, 4, index, 6);
  tri_mesh = dCreateTriMesh(game_space, mesh, 0, 0, 0);


  current->next = malloc(sizeof(struct TriColData));
  current = current->next;

  glColor3f(0.9, 0.9, 0.9);

  glNormal3f(normal[0], normal[1], normal[2]);
  glTexCoord2f(0.0,0.0);
  glVertex3f(f1 - road_width, f2, f3);
  glTexCoord2f(0.0,1.0);
  glVertex3f(f4 - road_width, f5, f6);
  glTexCoord2f(1.0,1.0);
  glVertex3f(f4 + road_width, f5, f6);

  glColor3f(0.9, 0.0, 0.0);

  glNormal3f(-1.0, 0.0, 0.0);
  glVertex3f(f1 - road_width, 0.0, f3);
  glVertex3f(f4 - road_width, 0.0, f6);
  glVertex3f(f4 - road_width, f5, f6);

  glColor3f(0.8, 0.0, 0.0);

  glVertex3f(f1 - road_width, 0.0, f3);
  glVertex3f(f1 - road_width, f2, f3);
  glVertex3f(f4 - road_width, f5, f6);

  glColor3f( 0.85, 0.85, 0.85);

  glNormal3f(normal[0], normal[1], normal[2]);
  glTexCoord2f(0.0,0.0);
  glVertex3f(f1 - road_width, f2, f3);
  glTexCoord2f(1.0,0.0);
  glVertex3f(f1 + road_width, f2, f3);
  glTexCoord2f(1.0,1.0);
  glVertex3f(f4 + road_width, f5, f6);

  glColor3f(0.9, 0.0, 0.0);

  glNormal3f(1.0, 0.0, 0.0);
  glVertex3f(f1 + road_width, 0.0, f3);
  glVertex3f(f4 + road_width, 0.0, f6);
  glVertex3f(f4 + road_width, f5, f6);

  glColor3f(0.8, 0.0, 0.0);

  glVertex3f(f1 + road_width, 0.0, f3);
  glVertex3f(f1 + road_width, f2, f3);
  glVertex3f(f4 + road_width, f5, f6);

}

void Quad(const float x1, const float y1, const float z1,
	  const float x2, const float y2, const float z2)
{
  /* Build a quad */
  glNormal3f(0.0, 1.0, 0.0);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(x1, y1, z1);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(x2, y1, z1);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(x2, y2, z2);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(x1, y2, z2);
}

GLuint BuildSkyBox(const float x1, const float y1, const float z1,
		   const float x2, const float y2, const float z2)
{
  /* Builds the skybox */
  GLuint display_list;

  #ifndef NODEBUG
  printf("Generating list\n(Skybox)\n");
  #endif

  display_list=glGenLists(1);

  #ifndef NODEBUG
  printf("Begin list.\n");
  #endif

  glNewList(display_list,GL_COMPILE);

  /* Begin the list */
  glBegin(GL_QUADS);

  #ifdef TEXTURE
  glColor3f(1.0, 1.0, 1.0);
  #else
  glColor3f(0.8,0.8,1.0);
  #endif


  Quad(x1, y1, z1, x2, y2, z1);

  Quad(x1, y1, z1, x2, y1, z2);
  Quad(x1, y2, z2, x2, y2, z1);

  /* End list creation */
  glEnd();

  #ifndef NODEBUG
  printf("End list.\n");
  #endif

  /* End the list */
  glEndList();

  #ifndef NODEBUG
  printf("List: %d.\n", display_list);
  #endif

  /* Return the display list */
  return display_list;
}

GLuint BuildGroundPlane(float x_start, float z_start,
			float dx, float dz,
			int x_segs, int z_segs)
{
  /* Builds a large rectangle at y=0 */

  /* Parameters */
  int i, j;
  float x1, z1, x2, z2;

  /* Pointer to the display list */
  GLuint display_list;

  #ifndef NODEBUG
  printf("Generating list\n(Ground plane)\n");
  #endif

  /* Allocate memory for the display list */
  display_list=glGenLists(1);

  /* Begin the display list */
  glNewList(display_list,GL_COMPILE);

  #ifndef NODEBUG
  printf("Begin list.\n");
  #endif

  z1 = z_start;
  z2 = z_start+dz;

  /* Create the list */
  glBegin(GL_QUADS);

  #ifdef TEXTURE
  glColor3f(1.0,1.0,1.0);
  #else
  glColor3f(0.42,0.42,0.42);
  #endif

  for(i=0;i<z_segs;i++) {
    x1 = x_start;
    x2 = x_start + dx;

    for(j=0;j<x_segs;j++) {
      /* Build the next quad */
      Quad(x1, 0.0, z1, x2, 0.0, z2);

      x1 = x2;
      x2 += dx;
    }
    z1 = z2;
    z2 += dz;

  }

  /* End list creation */
  glEnd();

  #ifndef NODEBUG
  printf("End list.\n");
  #endif

  /* End the list */
  glEndList();

  #ifndef NODEBUG
  printf("List: %d.\n", display_list);
  #endif

  /* Return the display list */
  return display_list;

}


/* Return the 2nd-order Lagrange polynomial */
float LagrangeP(float x,
	float x1, float x2, float x3,
	float y1, float y2, float y3)
{
  /*
   * P = ((x-x2)(x-x3) / (x1-x2)(x1-x3))y1 +
   *     ((x-x1)(x-x3) / (x2-x1)(x2-x3))y2 +
   *     ((x-x1)(x-x2) / (x3-x1)(x3-x2))y3
   */

  return ((((x - x2)*(x - x3)) / ((x1 - x2)*(x1 - x3))) * y1) +
    ((((x - x1)*(x - x3)) / ((x2 - x1)*(x2 - x3))) * y2) +
    ((((x - x1)*(x - x2)) / ((x3 - x1)*(x3 - x2))) * y3);
}


GLuint ReadSpline(char *filename)
{
  /* Reads the spline object specified by filename and builds the display list
     pointed to by display_list */

  /* File for input */
  FILE *file;

  int file_version;

  /* Parameters */
  float f1, f2, f3, f4, f5, f6, f7, f8, f9;
  float tf1, tf2, tf3, tf4, tf5, tf6, tfa;
  int i;

  /* Pointer to the display list */
  GLuint display_list;

  #ifndef NODEBUG
  printf("Generating list\n");

  /* Try and open the object file */
  printf("Opening spline file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /*
   * Validate the file.  Currently this consists of
   * checking it's a version 1.
   */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
    return 0;
  }

  /* Allocate memory for the display list */
  display_list=glGenLists(1);

  /* Begin the display list */
  glNewList(display_list,GL_COMPILE);

  #ifndef NODEBUG
  printf("Begin list.\n");
  #endif

  /* Create the list */
  glBegin(GL_TRIANGLES);

  fscanf(file, "%f %f %f\n", &f1, &f2, &f3);

  while(!feof(file)) {
    /* Input the next line of the file */
    fscanf(file, "%f %f %f\n", &f4, &f5, &f6);
    fscanf(file, "%f %f %f\n", &f7, &f8, &f9);

    tfa = 0.0;
    tf1 = LagrangeP(tfa, 0.0, 1.0, 2.0, f1, f4, f7);
    tf2 = LagrangeP(tfa, 0.0, 1.0, 2.0, f2, f5, f8);
    tf3 = LagrangeP(tfa, 0.0, 1.0, 2.0, f3, f6, f9);

    /* Subdivide the spline up */
    for (i=1;i<SPLINESTEPS-1;i++) {

      tfa = 0.0 + (2.0 * ((float)i / SPLINESTEPS));
      tf4 = LagrangeP(tfa, 0.0, 1.0, 2.0, f1, f4, f7);
      tf5 = LagrangeP(tfa, 0.0, 1.0, 2.0, f2, f5, f8);
      tf6 = LagrangeP(tfa, 0.0, 1.0, 2.0, f3, f6, f9);

      GenPolygon(tf1, tf2, tf3, tf4, tf5, tf6);
      tf1 = tf4; tf2 = tf5; tf3 = tf6;
    }

    f1 = f4; f2 = f5; f3 = f6;
  }



  /* End list creation */
  glEnd();

  #ifndef NODEBUG
  printf("End list.\n");
  #endif

  /* End the list */
  glEndList();

  #ifndef NODEBUG
  printf("List: %d.\n", display_list);
  #endif

  /* Close the input file */
  #ifndef NODEBUG
  printf("Closing spline file.\n");
  #endif

  fclose(file);

  /* Return the display list */
  return display_list;

}


void LoadPHGeneric(FILE *file, int obj_number)
{
    struct BasicProperties *prop;
    float v[3];
    dMass mass;
    dGeomID colbox;

    #ifndef NODEBUG
    printf("Physics model found: Generic object.\n");
    #endif

    obj[obj_number].physics_function = &PHGeneric;

    /* Allocate memory for physics structure */
    obj[obj_number].physics_data = malloc(sizeof(struct BasicProperties));

    prop = (struct BasicProperties *)obj[obj_number].physics_data;

    /* Read the physics data from the file */
    fscanf(file, "%f %f %f\n", &v[0], &v[1], &v[2]);
    fscanf(file, "%f\n", &prop->mass);

    /* Set up the ODE object */
    prop->id = dBodyCreate(game_world);

    dBodySetPosition(prop->id, obj[obj_number].x, obj[obj_number].y,
		     obj[obj_number].z);

    dMassSetZero(&mass);
    dMassSetBox(&mass, prop->mass, v[0], v[1], v[2]);

    dBodySetMass(prop->id, &mass);

    /* CHANGE THIS!!! */
    colbox = dCreateBox(0,v[0],v[1],v[2]);
    dGeomSetBody(colbox,prop->id);

    dSpaceAdd(game_space, colbox);

}

void LoadPHGenericSphere(FILE *file, int obj_number)
{
    struct BasicProperties *prop;
    float v;
    dMass mass;
    dGeomID colbox;

    #ifndef NODEBUG
    printf("Physics model found: Generic spherical object.\n");
    #endif

    obj[obj_number].physics_function = &PHGeneric;

    /* Allocate memory for physics structure */
    obj[obj_number].physics_data = malloc(sizeof(struct BasicProperties));

    prop = (struct BasicProperties *)obj[obj_number].physics_data;

    /* Read the physics data from the file */
    fscanf(file, "%f\n", &v);
    fscanf(file, "%f\n", &prop->mass);

    /* Set up the ODE object */
    prop->id = dBodyCreate(game_world);

    dBodySetPosition(prop->id, obj[obj_number].x, obj[obj_number].y,
		     obj[obj_number].z);

    dMassSetZero(&mass);
    dMassSetSphere(&mass, prop->mass, v);

    dBodySetMass(prop->id, &mass);

    /* CHANGE THIS!!! */
    colbox = dCreateSphere(0,v);
    dGeomSetBody(colbox,prop->id);

    dSpaceAdd(game_space, colbox);

}

void LoadPHCollisionGeometry(FILE *file, int obj_number)
{
  /* Build collision geometry for a static object */
  float x, y, z;
  char tstr[80];
  dGeomID colbox;

#ifndef NODEBUG
  printf("Physics model found: Collision geometry.\n");
#endif

  obj[obj_number].physics_function = &PHNothing;


  fscanf(file, "%s\n", tstr);
  if (strcmp(tstr, "BOX") == 0) {
    fscanf(file, "%f %f %f\n", &x, &y, &z);

    colbox = dCreateBox(0,x,y,z);
    dGeomSetPosition(colbox, obj[obj_number].x, obj[obj_number].y,
		     obj[obj_number].z);
    dGeomSetRotation(colbox, obj[obj_number].r);

    dSpaceAdd(game_space, colbox);
  }


}


void LoadWheel(FILE *file, struct WheelProperties *wheel, int obj_number)
{
  dMass mass;

  /* Zero the rotation and speed values */
  wheel->ry = 0.0;
  wheel->rx = 0.0;
  wheel->w = 0.0;
  wheel->torque = 0.0;
  wheel->col = current_wheel;
  wheel->tanSA = 0.0;

  /* Read the wheel data */
  fscanf(file, "%f %f %f\n", &wheel->r[0],
	 &wheel->r[1], &wheel->r[2]);
  fscanf(file, "%f %f\n", &wheel->mass,
	 &wheel->radius);
  fscanf(file, "%d\n", &wheel->driven);
  fscanf(file, "%f\n", &wheel->s_lock);
  fscanf(file, "%f\n", &wheel->susp_stiffness);

  fscanf(file, "%f\n", &wheel->brake_force);

  /* Read the tyre characteristics */

  fscanf(file, "%f %f %f\n", &wheel->lat.A,
	 &wheel->lat.B, &wheel->lat.P);
  fscanf(file, "%f %f %f\n", &wheel->lng.A,
	 &wheel->lng.B, &wheel->lng.P);

  fscanf(file, "%f %f\n", &wheel->optSA, &wheel->optSR);


  /* Set up the ODE object for the wheel */
  wheel->id = dBodyCreate(game_world);

  dBodySetPosition(wheel->id,
		   obj[obj_number].x + wheel->r[0],
		   obj[obj_number].y + wheel->r[1],
		   obj[obj_number].z + wheel->r[2]);

  dBodySetLinearVel(wheel->id, 0.0, 0.0, 0.0);


  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, wheel->mass / 100.0,
		      wheel->radius);
  dBodySetMass(wheel->id, &mass);

  /* Add the collision detection */
  tyre_col[current_wheel].body = dCreateSphere(0,wheel->radius);
  dGeomSetBody(tyre_col[current_wheel].body,wheel->id);

  dSpaceAdd(game_space, tyre_col[current_wheel].body);

  /* Damage properties */
  wheel->damage = 0.0;
  wheel->broken = 0;

  current_wheel++;

}

void LoadPHTrailer2Wheel(FILE *file, int obj_number)
{
  /* Load a simple, 2 wheel no-suspension trailer */
  struct Trailer2WheelProperties *prop;

  dMass mass;
  dGeomID colbox;

  int i, attach_to;
  float attach_pos[3];
  float f1, f2, f3;

#ifndef NODEBUG
  printf("Physics model found: Simple Trailer - 2 Wheel.\n");
#endif

    obj[obj_number].physics_function = &PHTrailer2Wheel;

    /* Allocate memory for physics structure */

    obj[obj_number].physics_data =
      malloc(sizeof(struct Trailer2WheelProperties));

    prop = (struct Trailer2WheelProperties *)obj[obj_number].physics_data;

    /* Read the physics data from the file */
    fscanf(file, "%f %f %f\n", &prop->length, &prop->width, &prop->height);
    fscanf(file, "%f\n", &prop->mass);
    fscanf(file, "%f %f %f\n", &f1, &f2, &f3);

    fscanf(file, "%d\n", &attach_to);
    fscanf(file, "%f %f %f\n", &attach_pos[0], &attach_pos[1], &attach_pos[2]);

#ifndef NODEBUG
    printf("Setting up the trailer body.\n");
#endif

    /* Set up the ODE object for the trailer body */
    prop->id = dBodyCreate(game_world);

    dBodySetPosition(prop->id, obj[obj_number].x, obj[obj_number].y,
		     obj[obj_number].z);
    dBodySetLinearVel(prop->id, 0.0, 0.0, 0.0);

    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, prop->mass / 100.0,
		     prop->width, prop->height, prop->length);
    dMassTranslate(&mass, f1, f2, f3);
    dBodySetMass(prop->id, &mass);

    /* Create the collision geometry */
    colbox = dCreateBox(0,prop->width,prop->height,prop->length);
    dGeomSetBody(colbox,prop->id);
    dSpaceAdd(game_space, colbox);

    /* Wheels */
    for (i=0;i<2;i++) {
      float K, C, erp, cfm;
      const float h = 0.015;

      LoadWheel(file, &prop->wheel[i], obj_number);

      /* Joint0r it to the body */
      prop->susp[i] = dJointCreateSlider(game_world, 0);
      dJointAttach(prop->susp[i], prop->id, prop->wheel[i].id);
      dJointSetSliderAxis(prop->susp[i], 0, 1, 0);

      dJointSetSliderParam(prop->susp[i], dParamLoStop, 0.0);
      dJointSetSliderParam(prop->susp[i], dParamHiStop, 0.0);

      /* Set suspension - always critically damped */
      K = prop->wheel[i].susp_stiffness;
      C = 2.0 * sqrt(K * (prop->mass / 100.0));
      erp = (h*K) / ((h*K) + C);
      cfm = 1.0 / ((h*K) + C);

      dJointSetSliderParam(prop->susp[i], dParamStopERP, erp);
      dJointSetSliderParam(prop->susp[i], dParamStopCFM, cfm);


    }

    /* Joint the trailer to the car */

    prop->attached = 1;

    if (obj[attach_to].physics_function == &PHCar) {
      struct CarProperties *car;

      #ifndef NODEBUG
      printf("Attaching to a car\n");
      #endif

      car = (struct CarProperties *)obj[attach_to].physics_data;

      prop->joint = dJointCreateBall(game_world, 0);
      dJointAttach(prop->joint, prop->id, car->id);
      dJointSetBallAnchor(prop->joint,
			  obj[obj_number].x + attach_pos[0],
			  obj[obj_number].y + attach_pos[1],
			  obj[obj_number].z + attach_pos[2]);

    } else if (obj[attach_to].physics_function == &PHTrailer2Wheel) {
      struct Trailer2WheelProperties *trailer;
      #ifndef NODEBUG
      printf("Attaching to another trailer.\n");
      #endif

      trailer = (struct Trailer2WheelProperties *)obj[attach_to].physics_data;

      prop->joint = dJointCreateBall(game_world, 0);
      dJointAttach(prop->joint, prop->id, trailer->id);
      dJointSetBallAnchor(prop->joint,
			  obj[obj_number].x + attach_pos[0],
			  obj[obj_number].y + attach_pos[1],
			  obj[obj_number].z + attach_pos[2]);


    }


}

void LoadPHWheel(FILE *file, int obj_number)
{
  struct WheelRepProperties *prop;
  int c, w;

  #ifndef NODEBUG
  printf("Physics model found: Car Wheel.\n");
  #endif

  /*
   * The physics data is simply a pointer to a wheel member of
   * a car structure.
   */
  obj[obj_number].physics_function = &PHWheel;

  /* Allocate memory */
  obj[obj_number].physics_data = malloc(sizeof(struct WheelRepProperties));
  prop = (struct WheelRepProperties *)obj[obj_number].physics_data;

  /* Load from file the object number of the car, and the wheel number */
  fscanf(file, "%d %d\n", &c, &w);

  /* Set the physics data to the relevant wheel */

  prop->car = &obj[c];

  if (obj[c].physics_function == &PHCar) {
    struct CarProperties *car;

    #ifndef NODEBUG
    printf("Representing a car wheel.\n");
    #endif
    car = (struct CarProperties *) obj[c].physics_data;
    prop->wheel = &car->wheel[w];

    /* Car wheels have their own drawing function */
    obj[obj_number].display_function = &GFXCarWheel;

  } else if (obj[c].physics_function == &PHTrailer2Wheel) {
    struct Trailer2WheelProperties *trailer;

    #ifndef NODEBUG
    printf("Representing a trailer wheel.\n");
    #endif
    trailer = (struct Trailer2WheelProperties *) obj[c].physics_data;
    prop->wheel = &trailer->wheel[w];

  }


}

void LoadPHFollowObj(FILE *file, int obj_number)
{
  struct FollowerProperties *prop;
  struct CarProperties *cprop;
  int c;

  #ifndef NODEBUG
  printf("Physics model found: Follower Object.\n");
  #endif

  /*
   * The physics data is simply a pointer to a body ID
   * a car structure.
   */
  obj[obj_number].physics_function = &PHFollow;

  /* Allocate memory */
  obj[obj_number].physics_data = malloc(sizeof(struct WheelRepProperties));
  prop = (struct FollowerProperties *)obj[obj_number].physics_data;

  /* Load from file the object number to follow */
  fscanf(file, "%d\n", &c);

  /* WARNING: ASSUMES THE CAR IS ALWAYS OBJECT 0 ! */

  /* 1-4 = wheels, 5-8 = front suspension struts, 9,10 = front susp spindles
     11 = diff 12 = rear axle */
  cprop = (struct CarProperties *)obj[0].physics_data;

  switch(c) {
  case 1:
    prop->object = cprop->wheel[0].id;
    break;
  case 2:
    prop->object = cprop->wheel[1].id;
    break;
  case 3:
    prop->object = cprop->wheel[2].id;
    break;
  case 4:
    prop->object = cprop->wheel[3].id;
    break;
  default:
    /* Giv0r up and use the car instead */
    prop->object = cprop->id;
  }

}

void LoadPHCar(FILE *file, const int obj_number)
{
    struct CarProperties *prop;

    FILE *car_file;

    char tstr[80];
    dMass mass;
    dGeomID colbox;

    int i;
    int e_cyls;
    float e_cyl_offset, e_v_open, e_v_dur, e_v_pressure;
    float cg_y, cg_z;

    #ifndef NODEBUG
    printf("Physics model found: Car.\n");
    #endif

    obj[obj_number].physics_function = &PHCar;

    /* Allocate memory for physics structure */
    obj[obj_number].physics_data = malloc(sizeof(struct CarProperties));

    prop = (struct CarProperties *)obj[obj_number].physics_data;

    fscanf(file, "%s\n", tstr);

    /* Open the car physics file */
    #ifndef NODEBUG
    printf("Opening car physics file %s.\n", tstr);
    #endif
    car_file = fopen(tstr, "r");

    /* Read the physics data from the file */
    fscanf(car_file, "%f %f %f\n", &prop->length, &prop->width, &prop->height);
    fscanf(car_file, "%f\n", &prop->mass);
    fscanf(car_file, "%f %f %f\n", &prop->inertia[0], &prop->inertia[1],
	   &prop->inertia[2]);
    fscanf(car_file, "%f %f\n", &cg_y, &cg_z);

    /* Set up the ODE object for the sprung mass */
    prop->id = dBodyCreate(game_world);

    dBodySetPosition(prop->id, obj[obj_number].x, obj[obj_number].y,
		     obj[obj_number].z);
    dBodySetLinearVel(prop->id, 0.0, 0.0, 0.0);

    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, prop->mass / 100.0,
		     prop->length, prop->height/2.0, prop->width);


    dBodySetMass(prop->id, &mass);


    /* CHANGE THIS!!! */
    colbox = dCreateBox(0,prop->length,prop->height,prop->width);
    dGeomSetBody(colbox,prop->id);
    dSpaceAdd(game_space, colbox);

    car_body = colbox;


    /* Wheels */
    for (i=0;i<4;i++) {
      float erp, cfm, K, C;
      const float h = 0.015;

      LoadWheel(car_file, &prop->wheel[i], obj_number);

      /* Joint0r it to the body */
      prop->susp[i] = dJointCreateSlider(game_world, 0);
      dJointAttach(prop->susp[i], prop->id, prop->wheel[i].id);

      /*
       * Suspension geometry is modelled - wheelbase and track increase
       * as suspension compresses
       */
      dJointSetSliderAxis(prop->susp[i], prop->wheel[i].r[0] / 10.0,
			  1, prop->wheel[i].r[2] / 10.0);

      dJointSetSliderParam(prop->susp[i], dParamLoStop, 0.0);
      dJointSetSliderParam(prop->susp[i], dParamHiStop, 0.0);

      /* Set suspension - always critically damped */
      K = prop->wheel[i].susp_stiffness;
      C = 2.0 * sqrt(K * (prop->mass / 100.0));
      erp = (h*K) / ((h*K) + C);
      cfm = 1.0 / ((h*K) + C);

      dJointSetSliderParam(prop->susp[i], dParamStopERP, erp);
      dJointSetSliderParam(prop->susp[i], dParamStopCFM, cfm);



    }

    /* Gearbox data */

    /* Default */
    for(i=0;i<10;i++) {
      prop->gearbox.ratio[i] = 0.0;
    }

    fscanf(car_file, "%u\n", &prop->gearbox.num_gears);

    for(i=0;i<prop->gearbox.num_gears;i++) {
      fscanf(car_file, "%f\n", &prop->gearbox.ratio[i]);
    }


    fscanf(car_file, "%f %f\n", &prop->engine.inertia,
	   &prop->engine.max_torque);

    prop->engine.max_torque = prop->engine.max_torque;
    prop->engine.inertia = prop->engine.inertia;

    fscanf(car_file, "%f %f %f\n", &prop->engine.max_rpm,
	   &prop->engine.idle_rpm, &prop->engine.idle_throttle);
    prop->engine.rpm = prop->engine.idle_rpm;


    /* Read the engine data */

    /* Torque curve */
    for (i=0;i<10;i++) {
      fscanf(car_file, "%f", &prop->engine.torque_curve[i]);
    }

    fscanf(car_file, "%d\n", &e_cyls);

    SetCylinders(e_cyls);

    for(i=0;i<e_cyls;i++) {
      fscanf(car_file, "%f\n", &e_cyl_offset);
      SetCylOffset(i, e_cyl_offset);
    }

    for (i=0;i<4;i++) {
      fscanf(car_file, "%f %f %f\n", &e_v_open, &e_v_dur, &e_v_pressure);
      SetValveOpenPos(i, e_v_open);
      SetValveOpenDuration(i, e_v_dur);
      SetValvePressureDrop(i, e_v_pressure);
    }


    fscanf(car_file, "%f %f %f\n", &prop->exhaust_pos[0],
	   &prop->exhaust_pos[1], &prop->exhaust_pos[2]);


    /* Read the car light positions */
    for(i=0;i<2;i++) {
      fscanf(car_file, "%f %f %f\n", &prop->brake_light_pos[i][0],
	     &prop->brake_light_pos[i][1], &prop->brake_light_pos[i][2]);
    }

    for(i=0;i<2;i++) {
      fscanf(car_file, "%f %f %f\n", &prop->reverse_light_pos[i][0],
	     &prop->reverse_light_pos[i][1], &prop->reverse_light_pos[i][2]);
    }

    for(i=0;i<2;i++) {
      fscanf(car_file, "%f %f %f\n", &prop->indicator_pos[0][i][0],
	     &prop->indicator_pos[0][i][1], &prop->indicator_pos[0][i][2]);
    }
    for(i=0;i<2;i++) {
      fscanf(car_file, "%f %f %f\n", &prop->indicator_pos[1][i][0],
	     &prop->indicator_pos[1][i][1], &prop->indicator_pos[1][i][2]);
    }

    fclose(car_file);


    gear = 0;

    /* Kludge to drop CoG */
    dMassTranslate(&mass, 0.0, cg_y, cg_z);


    /* Cars have their own drawing function */
    obj[obj_number].display_function = &GFXCar;

}

void LoadPHArtifactSpawn(FILE *file, int obj_number)
{
  struct ArtifactSpawnProperties *prop;

#ifndef NODEBUG
  printf("Physics model found: Artifact Spawn.\n");
#endif

  obj[obj_number].physics_function = &PHArtifactSpawn;

  obj[obj_number].physics_data =
    malloc(sizeof(struct ArtifactSpawnProperties));

  prop = (struct ArtifactSpawnProperties *)obj[obj_number].physics_data;

  /* Get all the data from the file */

  fscanf(file, "%f %f %f\n", &prop->x, &prop->y, &prop->z);
  fscanf(file, "%f %f %f\n", &prop->dx, &prop->dy, &prop->dz);
  fscanf(file, "%f %f %f\n", &prop->vdx, &prop->vdy, &prop->vdz);

  fscanf(file, "%f %f %f\n", &prop->ax, &prop->ay, &prop->az);

  fscanf(file, "%f %f %f\n", &prop->r, &prop->g, &prop->b);
  fscanf(file, "%f %f %f\n", &prop->vr, &prop->vg, &prop->vb);

  fscanf(file, "%f %f %f\n", &prop->size, &prop->growth, &prop->vgrowth);
  fscanf(file, "%f %f %f\n", &prop->life, &prop->decay, &prop->vdecay);

  fscanf(file, "%d %d\n", &prop->frequency, &prop->spawned);
  fscanf(file, "%d\n", &prop->lit);

}

void LoadPHGauge(FILE *file, int obj_number)
{
    struct GaugeProperties *prop;
    int i;

    #ifndef NODEBUG
    printf("Physics model found: Gauge.\n");
    #endif

    /* Tell the program not to translate or rotate this object */
    obj[obj_number].no_translate = 1;

    obj[obj_number].physics_function = &PHGauge;

    /* Allocate memory for physics structure */
    obj[obj_number].physics_data = malloc(sizeof(struct GaugeProperties));

    prop = (struct GaugeProperties *)obj[obj_number].physics_data;

    /* Read the physics data from the file */
    fscanf(file, "%d\n", &i);
    fscanf(file, "%d\n", &prop->type);
    fscanf(file, "%f %f\n", &prop->min, &prop->max);
    fscanf(file, "%f %f\n", &prop->min_rot, &prop->max_rot);

    /* Mode - 0 = RPM, 1 = speed */
    fscanf(file, "%d\n", &prop->mode);

}


void ReadEntity(char *filename, int obj_number)
{
  /* Read an entity from an entity file
   *
   * Entity files contain an object filename to load, a type of physics
   * for that object, and also any members of the physics data that the
   * object can have
   */

  /* File for input */
  FILE *file;

  int file_version, i;

  /* Parameters */
  float f1, f2, f3;

  /* String */
  char tstr[80];

  /* Try and open the object file */
  #ifndef NODEBUG
  printf("Opening entity file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /* Validate the file.  Currently this consists of checking it's a version 1. */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
  }

  /* Load the object
   *
   * If the object filename is EXISTING then the object should use an
   * existing display list, else we should read from the filename given.
   * If we have the special case TEXTURED then this object has a texture
   * map associated with it as well, and this should be dealt with before
   * loading the object.  Also, now there is the option of ROADSPLINE
   * which is yet another thing
   */
  fscanf(file, "%s\n", tstr);

  obj[obj_number].texture = 0;
  obj[obj_number].no_translate = 0;


  #ifndef NODEBUG
  printf("Setting display function.\n");
  #endif

  obj[obj_number].display_function = &GFXDisplayList;


  if (strcmp(tstr, "TEXTURED") == 0) {
    /* Read the texture this object will use */
    fscanf(file, "%d\n", &obj[obj_number].texture);

    /* The last thing we should do is again read the next line */
    fscanf(file, "%s\n", tstr);
  }

  if (strcmp(tstr, "EXISTING") == 0) {
    fscanf(file, "%d\n", &i);

    #ifndef NODEBUG
    printf("Using existing display list from object %d.\n", i);
    #endif

    obj[obj_number].display_list = obj[i].display_list;

  } else if (strcmp(tstr, "ROADSPLINE") == 0) {
    #ifndef NODEBUG
    printf("Reading road spline.\n");
    #endif

    /* Read the spline file */
    fscanf(file, "%s\n", tstr);

    obj[obj_number].display_list = ReadSpline(tstr);

  } else if (strcmp(tstr, "GROUNDPLANE") == 0) {
    float x1, z1, dx, dz;
    int n_x, n_z;

    #ifndef NODEBUG
    printf("Building ground plane.\n");
    #endif

    /* Read the parameters */
    fscanf(file, "%f %f %f %f\n", &x1, &z1, &dx, &dz);
    fscanf(file, "%d %d\n", &n_x, &n_z);

    obj[obj_number].display_list = BuildGroundPlane(x1,z1,dx,dz,n_x,n_z);
  } else if (strcmp(tstr, "SKYBOX") == 0) {
    float x1, x2, y1, y2, z1, z2;

    #ifndef NODEBUG
    printf("Building skybox.\n");
    #endif

    /* Read the parameters */
    fscanf(file, "%f %f %f\n", &x1, &y1, &z1);
    fscanf(file, "%f %f %f\n", &x2, &y2, &z2);

    obj[obj_number].display_list = BuildSkyBox(x1,y1,z1,x2,y2,z2);
    obj[obj_number].no_translate = 1;

  } else if (strcmp(tstr, "NONE") == 0) {
    obj[obj_number].display_function = GFXNone;
  } else {
    #ifndef NODEBUG
    printf("Loading object file %s.\n", tstr);
    #endif

    obj[obj_number].display_list = ReadObject(tstr);
  }

  #ifndef NODEBUG
  printf("Loading position data.\n");
  #endif

  fscanf(file, "%f %f %f\n", &f1, &f2, &f3);
  obj[obj_number].x = f1;
  obj[obj_number].y = f2;
  obj[obj_number].z = f3;

  #ifndef NODEBUG
  printf("Loading rotation data.\n");
  #endif

  fscanf(file, "%f %f %f\n", &f1, &f2, &f3);

  /* Set initial values in the rotation matrix */
  GenerateRotationMatrix(obj[obj_number].r, f1, f2, f3);

  /* Load the object's physics routine */
  fscanf(file, "%s\n", tstr);

  #ifndef NODEBUG
  printf("Physics model in file: %s.\n", tstr);
  #endif

  /* Long multi-if statement to load the physics data */
  if (strcmp(tstr, "PHGeneric") == 0) {
    LoadPHGeneric(file, obj_number);
  } else if (strcmp(tstr, "PHGenericSphere") == 0) {
    LoadPHGenericSphere(file, obj_number);
  } else if (strcmp(tstr, "PHCar") == 0) {
    LoadPHCar(file, obj_number);
  } else if (strcmp(tstr, "PHTrailer2Wheel") == 0) {
    LoadPHTrailer2Wheel(file, obj_number);
  } else if (strcmp(tstr, "PHGauge") == 0) {
    LoadPHGauge(file, obj_number);
  } else if (strcmp(tstr, "PHWheel") == 0) {
    LoadPHWheel(file, obj_number);
  } else if (strcmp(tstr, "PHFollower") == 0) {
    LoadPHFollowObj(file, obj_number);
  } else if (strcmp(tstr, "PHCollisionGeometry") == 0) {
    LoadPHCollisionGeometry(file, obj_number);
  } else if (strcmp(tstr, "PHArtifactSpawn") == 0) {
    LoadPHArtifactSpawn(file, obj_number);
  } else {
    #ifndef NODEBUG
    printf("Using null physics function.\n");
    #endif

    obj[obj_number].physics_function = &PHNothing;
  }


  /* Close the input file */
  #ifndef NODEBUG
  printf("Closing entity file.\n");
  #endif

  fclose(file);
}

void LoadCamera(char *filename, int cam_number)
{
  /* Loads and sets up a camera */

  /* Read an entity from an entity file
   *
   * Entity files contain an object filename to load, a type of physics
   * for that object, and also any members of the physics data that the
   * object can have
   */

  /* File for input */
  FILE *file;

  int file_version;

  float rx, ry, rz;

  /* String */
  char tstr[80];

  /* Try and open the object file */
  #ifndef NODEBUG
  printf("Opening camera entity file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /* Validate the file.  Currently this consists of checking it's a
     version 1. */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
  }

  fscanf(file, "%s\n", tstr);
  if (strcmp(tstr, "CAMERA") != 0) {
    printf("Entity is not a camera.\n");
  }

  /* Load the linked object */
  fscanf(file, "%d\n", &camera[cam_number].object);

  #ifndef NODEBUG
  printf("Linking camera to object %d.\n", camera[cam_number].object);

  /* Load the position and rotation offsets */
  printf("Loading position and rotation data.\n");
  #endif

  fscanf(file, "%f %f %f\n", &camera[cam_number].x, &camera[cam_number].y,
	 &camera[cam_number].z);
  fscanf(file, "%f %f %f\n", &rx, &ry, &rz);

  GenerateRotationMatrix(camera[cam_number].r, rx, ry, rz);

  /* Load the tracking information */
  fscanf(file, "%u %u %u\n", &camera[cam_number].track_x,
	 &camera[cam_number].track_y, &camera[cam_number].track_z);
  fscanf(file, "%u\n", &camera[cam_number].track_r);
  fscanf(file, "%u\n", &camera[cam_number].track_mode);
  fscanf(file, "%u\n", &camera[cam_number].interior);

  /* Close the input file */
  #ifndef NODEBUG
  printf("Closing camera entity file.\n");
  #endif

  fclose(file);
}

void LoadLight(char *filename, int l)
{
  FILE *file;

  int file_version;

  /* Try and open the object file */
  #ifndef NODEBUG
  printf("Opening lighting entity file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /* Validate the file.  Currently this consists of checking it's a
     version 1. */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
  }

  fscanf(file, "%f %f %f %f\n", &light[l].position[0],
	 &light[l].position[1],
	 &light[l].position[2],
	 &light[l].position[3]);

  fscanf(file, "%f %f %f %f\n", &light[l].ambient[0],
	 &light[l].ambient[1],
	 &light[l].ambient[2],
	 &light[l].ambient[3]);

  fscanf(file, "%f %f %f %f\n", &light[l].diffuse[0],
	 &light[l].diffuse[1],
	 &light[l].diffuse[2],
	 &light[l].diffuse[3]);

  fscanf(file, "%f %f %f\n", &light[l].attenuation[0],
	 &light[l].attenuation[1],
	 &light[l].attenuation[2]);


  /* Does the light flicker? */
  fscanf(file, "%d\n", &light[l].flicker);

  fscanf(file, "%f %f %f %f\n", &light[l].flicker_colour[0],
	 &light[l].flicker_colour[1], &light[l].flicker_colour[2],
	 &light[l].flicker_colour[3]);

  fscanf(file, "%d\n", &light[l].follow_car);

  if(light[l].follow_car) {
    /* Set up the car following stats */
    int car_to_follow;

    fscanf(file, "%d\n", &car_to_follow);
    light[l].car = obj[car_to_follow].physics_data;

    fscanf(file, "%f %f %f\n", &light[l].pos[0],
	   &light[l].pos[1], &light[l].pos[2]);
  }

  light[l].last_flickered = 0;

  /* Lighting stuff */
  #ifdef LIGHTING
  glLightfv(GL_LIGHT0 + l, GL_AMBIENT, light[l].ambient);
  glLightfv(GL_LIGHT0 + l, GL_DIFFUSE, light[l].diffuse);
  glLightfv(GL_LIGHT0 + l, GL_POSITION, light[l].position);
  glLightfv(GL_LIGHT0 + l, GL_CONSTANT_ATTENUATION, &light[l].attenuation[0]);
  glLightfv(GL_LIGHT0 + l, GL_LINEAR_ATTENUATION, &light[l].attenuation[1]);
  glLightfv(GL_LIGHT0 + l, GL_QUADRATIC_ATTENUATION, &light[l].attenuation[2]);
  glEnable(GL_LIGHT0 + l);
  #endif

  #ifndef NODEBUG
  printf("Closing lighting entity file.\n");
  #endif

  fclose(file);

}


void ReadScene(char *filename)
{
  /*
   * Read a game scene from a scene file.
   *
   * A scene file is basically a set of entity files, all if which put together
   * make up the level/game/whatever the engine is going to run.
   */

  /* File for input */
  FILE *file;

  int file_version, i, entities;
  float f[4];

  /* String */
  char tstr[80];

  detach_trailers = 0;

  /* Create the ODE world */
  #ifndef NODEBUG
  printf("Creating the game world\n");
  #endif
  game_world = dWorldCreate();


  /* Create a collision detection geometry array */
  first = malloc(sizeof(struct TriColData));
  current = first;

  /* Create the collision space */
  game_space = dHashSpaceCreate(0);

  /* Create the collision joint group */
  contact_group = dJointGroupCreate(0);

  /* Set the gravity */
  dWorldSetGravity(game_world, 0.0, -G, 0.0);

  /* Create the ground plane - TEMP */
  dCreatePlane(game_space,0.0,1.0,0.0,0.0);

  /* Try and open the scene file */
  #ifndef NODEBUG
  printf("Opening scene file %s.\n", filename);
  #endif

  file = fopen(filename, "r");

  /* Validate the file.  Currently this consists of checking it's a version
     1. */
  fscanf(file, "%d\n", &file_version);

  if (file_version != 1) {
    printf("Incorrect file version.\n");
  }

  /* Read and set the window caption */
  fgets(tstr, 80, file);
  tstr[strlen(tstr) - 1] = 0;
  SDL_WM_SetCaption(tstr, "");

  /* Read the number of wheels in the scene */
  fscanf(file, "%d\n", &num_wheels);

  /* Allocate space for the wheels array */
  tyre_col = malloc(num_wheels * sizeof(struct TyreCollisionProperties));
  for(i=0;i<num_wheels;i++) {
    tyre_col[i].contact = 0;
    tyre_col[i].load = 0;
  }


  /* Read the number of texture entities */
  fscanf(file, "%d\n", &entities);
  artifact_texture = entities - 1;
  car_int_texture = entities - 2;

  /* Allocate texture memory */
  texture = (unsigned int *) malloc(entities * sizeof(int));

  /* Load each texture */
  for (i=0;i<entities;i++) {
    fscanf(file, "%s\n", tstr);

    #ifndef NODEBUG
    printf("Loading texture file %s.\n", tstr);
    #endif

    /* Load the textures */
    LoadGLTexture(tstr, i);
  }

  /* Read the number of object entities */
  fscanf(file, "%d\n", &entities);
      scn_objects = entities;

  /* Allocate object memory */
  obj = (struct Object3D *) malloc(entities * sizeof(struct Object3D));

  /* Load each entity */
  for (i=0;i<entities;i++) {
    fscanf(file, "%s\n", tstr);

    #ifndef NODEBUG
    printf("Loading entity file %s.\n", tstr);
    #endif

    ReadEntity(tstr, i);
  }

  /* Read the number of camera entities */
  fscanf(file, "%d\n", &cameras);

  /* Allocate camera memory */
  camera = (struct Camera3D *) malloc(entities * sizeof(struct Camera3D));

  /* Load each camera */
  for (i=0;i<cameras;i++) {
    fscanf(file, "%s\n", tstr);

    #ifndef NODEBUG
    printf("Loading camera file %s.\n", tstr);
    #endif

    LoadCamera(tstr, i);
  }

  /* Read the number of lights */
  fscanf(file, "%d\n", &lights);

  /* Allocate lighting memory */
  light = (struct Light3D *) malloc(lights * sizeof(struct Light3D));

  /* Get the ambient light level */
  fscanf(file, "%f %f %f\n", &f[0], &f[1], &f[2]);
  f[3] = 1.0;

  /* Tell OpenGL what the ambient light level should be */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, f);

  /* Load each light */
  for (i=0;i<lights;i++) {
    fscanf(file, "%s\n", tstr);

    #ifndef NODEBUG
    printf("Loading light file %s.\n", tstr);
    #endif

    LoadLight(tstr, i);
  }


  /* Read the music for this level */
  fscanf(file, "%s\n", tstr);

  #ifndef NODEBUG
  printf("Playing music file %s.\n", tstr);
  #endif

  Snd_LoadMP3(tstr);

  /* Close the input file */
  #ifndef NODEBUG
  printf("Closing scene file.\n");
  #endif

  fclose(file);

  /* TEMP */
  gear = 1;


  /* Initialise the artifacts */
  InitArtifacts();


}

void HandleJoystickAxis(SDL_JoyAxisEvent joy)
{
  switch(joy.axis) {
  case 0:
    /* Steering */
    axis[0] = (float)joy.value / 32767.0;
    break;
  case 1:
    /* Throttle and brake */
    if (joy.value < 0) {
      axis[1] = (float)joy.value / -32767.0;
      axis[2] = 0.0;
    } else {
      axis[1] = 0.0;
      axis[2] = (float)joy.value / 32767.0;
    }
    break;
  }
}

void HandleJoystickButton(SDL_JoyButtonEvent joy)
{
  switch(joy.button) {
  case 0:
  case 8:
    gear++;
    gear_changed = 1;
    break;
  case 1:
  case 9:
    gear--;
    gear_changed = 1;
    break;
  }
}

void HandleKeyUp(SDLKey sym)
{
  /*
   * Handle key-up events for the keyboard handler - this is so we
   * can have control systems in which the user can hold down a key
   */

  switch(sym) {
#ifndef JOYSTICK
  case SDLK_LEFT:
    d_axis[0] = 0.0;
    break;
  case SDLK_RIGHT:
    d_axis[0] = 0.0;
    break;
  case SDLK_UP:
    d_axis[1] = 0.0;
    break;
  case SDLK_DOWN:
    d_axis[2] = 0.0;
    break;
#endif
  default:
    break;
  }
}

void HandleKeyDown(SDLKey sym)
{
  /*
   * Handle key presses - this can be as part of the user holding
   * the key down, or as an instantaneous keypress which shouldn't
   * be repeated.
   */

  switch(sym) {
#ifndef JOYSTICK
  case SDLK_LEFT:
    d_axis[0] = -0.007;
    break;
  case SDLK_RIGHT:
    d_axis[0] = 0.007
;
    break;
  case SDLK_UP:
    d_axis[1] = 1.0;
    d_axis[2] = -1.0;
    break;
  case SDLK_DOWN:
    d_axis[1] = -1.0;
    d_axis[2] = 1.0;
    break;
  case SDLK_a:
    gear++;
    gear_changed = 1;
    break;
  case SDLK_z:
    gear--;
    gear_changed = 1;
    break;
#endif

  case SDLK_c:
    camera_ry = 0.0;
    cur_cam = (cur_cam + 1) % cameras;
    break;
  case SDLK_m:
    Snd_PlayPauseMusic();
    break;
  case SDLK_KP_MINUS:
    Snd_MusicVolumeDown();
    break;
  case SDLK_KP_PLUS:
    Snd_MusicVolumeUp();
    break;

  case SDLK_d:
    detach_trailers = 1;
    break;

  case SDLK_ESCAPE:
    /* Shut down window */
#ifndef NODEBUG
    printf("Exiting due to SDL keypress\n");
#endif
    Quit();
    break;
  default:
    break;
  }
}

void ProcessEvents()
{
  /* SDL Event */
  SDL_Event event;

  SDL_PumpEvents();

  /* Get all events from queue */

  while (SDL_PollEvent(&event)) {
    switch(event.type) {
    case SDL_KEYDOWN:
      /* Get events when a key goes down */
      HandleKeyDown(event.key.keysym.sym);
      break;
    case SDL_KEYUP:
      /* Get events when a key goes down */
      HandleKeyUp(event.key.keysym.sym);
      break;
    case SDL_VIDEORESIZE:
      /*
       * Handle resize event
       *
       * This needs more error checking for program stability.
       */
      SDL_SetVideoMode(event.resize.w, event.resize.h, bpp, flags);
      ResizeGLScene(event.resize.w, event.resize.h);
      break;
#ifdef JOYSTICK
    case SDL_JOYAXISMOTION:
      HandleJoystickAxis(event.jaxis);
      break;
    case SDL_JOYBUTTONDOWN:
      HandleJoystickButton(event.jbutton);
      break;
#endif
    case SDL_MOUSEMOTION:
      if(SDL_GetMouseState(NULL, NULL)&SDL_BUTTON(1)) {
      	camera_ry += (float)(event.motion.xrel) / 100.0;
      }
      break;
    case SDL_QUIT:
      Quit();
      break;
    default:
      break;
    }

  }

}


void GameLoop()
{
  int i = 0;

  while(1) {
    i++;
    ProcessEvents();
    DrawGLScene();
  }


}

int main(int argc, char *argv[])
{
  /* Information about the current video settings */
  const SDL_VideoInfo* info = NULL;

  /* Dimensions of the window */
  int width = 0, height = 0;

  /* Initialise SDL - video subystem */
  SDL_Init( SDL_INIT_AUDIO | SDL_INIT_VIDEO
#ifdef JOYSTICK
| SDL_INIT_JOYSTICK
#endif
	    );


  /* Get video information */
  info = SDL_GetVideoInfo();

  /* Attributes for the display */
#ifdef FULLSCREEN
  width = 1024;
  height = 768;
  //width = 320;
  //height = 200;
#else
  width = 640;
  height = 480;
#endif

#ifdef LOWRES
  width = 320;
  height = 240;
#endif

  bpp = info->vfmt->BitsPerPixel;

  /* Set the GL attributes - minimum */
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

#ifdef FULLSCREEN
  flags = SDL_OPENGL | SDL_FULLSCREEN;
#else
  flags = SDL_OPENGL | SDL_RESIZABLE;
#endif

  /* OpenGL mode */
  SDL_SetVideoMode(width,height,bpp,flags);

  /* Set the program caption */
  SDL_WM_SetCaption("Generic Game Engine", "");

  /* Start the audio system */
  Snd_StartAudio();

  /* Level initialisation */
  /* Read the scene */
  if (argc > 1) {
    printf("attempt read scene %s\n", argv[1]);
    ReadScene(argv[1]);
  } else {
    printf("attempt read scene\n");
    ReadScene("assets/levels/title.scn");
  }

    printf("Scene read\n");

  /* Enable texture mapping and material colours */
  #ifdef TEXTURE
  glEnable(GL_TEXTURE_2D);
  #endif
  glEnable(GL_COLOR_MATERIAL);

  /* Initialise the window */
  InitGL(width,height);

  #ifndef NODEBUG
  printf("OpenGL initialised\n");
  #endif


  /* Load the joystick */
#ifdef JOYSTICK
  SDL_JoystickOpen(0);
  SDL_JoystickEventState(SDL_ENABLE);
#endif

  /* TEMP TODO sound */
  Snd_LoadSound("assets/sound/tyres.wav", 0);
  Snd_StartSampleLoop(0,0);
  Snd_AddEffect(0, 1);



  /* Start the timing info */
  UpdateTimingInfo();


  GameLoop();

  /* If we got here, it was a failure */
  return 1;
}

