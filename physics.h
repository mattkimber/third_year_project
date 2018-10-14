/* Physics header file */

#define PHYSICS

/* Anything which requires physics requires maths.*/
#include <math.h>
#include <ode/ode.h>

/* This is our own vectors header */
#include "vector.h"

/* Gravity */
#define G 0.981

/* Useful for angle conversion */
#define PI M_PI
#define DEG_TO_RAD(i) (i * PI) / 180.0

/* Maximum number of gears */
#define MAX_GEARS 8


#define SGN(f) f > 0.0 ? 1.0 : -1.0

/* String used for scoring */
char score_string[80];

/* The world ID of the ODE world */
dWorldID game_world;

/* Collision detection space */
dSpaceID game_space;

/* Joint used in collision detection */
dJointGroupID contact_group;

dJointID joint;

/* Wheel and car body IDs used as part of collision detection */
dGeomID car_body;

/* Information for the gauges */
float car_velocity;
float car_rpm;

/*
 * An object's collision detection data.
 * Basically, the radius of the bounding sphere,
 * and the triangles that make up its bounding shape
 */
struct CollisionData {
  /* Radius of the bounding sphere */
  float radius;

  /* Number of collision detection triangles */
  int num_triangles;

  /* Array of triangles that make up the bounding shape */
  float *t;
};


/* Basic properties for an object */
struct BasicProperties
{
  /* ODE body ID */
  dBodyID id;
  
  /* Mass and moment of inertia */
  float mass;

};

/* Follower object */
struct FollowerProperties
{
  /* ODE body ID */
  dBodyID object;
};

/* Brian Beckman's magic trick formula constants */
struct MagicTrickProperties
{
  /* Breakaway slip scaling */
  float A;
  
  /* Force scaling */
  float B;

  /* Breakaway slip exponent */
  float P;
};

/* Basic properties for a car wheel */
struct WheelProperties
{
  /* ODE body ID */
  dBodyID id;

  /* Location relative to the car as an xyz triple */
  float r[3];

  /* Mass */
  float mass;

  /* Radius */
  float radius;

  /* Steering angle */
  float ry;

  /* Angle turned */
  float rx;

  /* Rotational velocity */
  float w;

  /* Max steering lock */
  float s_lock;

  /* Torque on the wheel */
  float torque;

  /* Braking power */
  float brake_force;

  /* Suspension stiffness */
  float susp_stiffness;

  /* Damage */
  float damage;
  int broken;

  /* Power multiplier (how much power the wheel gets) */
  int driven;
  float p_mul;

  /* Which tyre collision structure is associated with this tyre */
  int col;

  /* Lateral and longitudinal magic trick co-efficients */
  struct MagicTrickProperties lng;
  struct MagicTrickProperties lat;

  /* Optimal slip angle and ratio */
  float optSA;
  float optSR;

  /* Current slip angle tangent */
  float tanSA;

};

struct GearboxProperties
{
  /* Number of gears */
  int num_gears;

  /* Ratios */
  float ratio[MAX_GEARS];
};

/* Properties for the wheel/ground collision */
struct TyreCollisionProperties
{
  int contact;
  float load;
  dGeomID body;
  dJointFeedback joint_info;
};

/* Basic properties for the representation of a wheel */
struct WheelRepProperties
{
  /* Pointer to the car (or trailer) */
  struct Object3D *car;

  /* The wheel */
  struct WheelProperties *wheel;
};

/* Properties of a car engine */
struct EngineProperties
{
  /* Inertia */
  float inertia;

  /* Constant torque */
  float max_torque;

  /* RPM limiter */
  float max_rpm;

  /* Idle RPM */
  float idle_rpm;

  /* Current RPM */
  float rpm;

  /* Driveshaft RPM */
  float shaft_rpm;

  /* Throttle amount for idling */
  float idle_throttle;

  /* Torque curve */
  float torque_curve[10];
};


/* Basic properties for a car */
struct CarProperties
{
  /* ODE body ID */
  dBodyID id;

  /* Dimensions */
  float length;
  float width;
  float height;

  /* Mass and inertia */
  float mass;
  float inertia[3];

  /* Joints for the suspension */
  dJointID susp[4];

  /* Wheels */
  struct WheelProperties wheel[4];

  /* Engine */
  struct EngineProperties engine;

  /* Gearbox */
  struct GearboxProperties gearbox;

  /* Exhaust smoke position */
  float exhaust_pos[3];

  /* Brake light position */
  float brake_light_pos[2][3];
  float reverse_light_pos[2][3];
  float indicator_pos[2][2][3];
};

struct Trailer2WheelProperties
{
  /* ODE body ID */
  dBodyID id;

  /* Dimensions */
  float length;
  float width;
  float height;
  
  /* Mass */
  float mass;

  /* Joint with the car/other trailer */
  dJointID joint;

  /* Wheel hinges - jokingly called "suspension" */
  dJointID susp[2];

  /* Is it attached */
  int attached;

  /* The wheels */
  struct WheelProperties wheel[2];

};

/* Basic properties for a car gauge */
struct GaugeProperties
{
  /* Type. */
  int type;

  /* Maximum and minimum values */
  float max;
  float min;

  /* Maximum and minimum rotations */
  float max_rot;
  float min_rot;

  /* Type of gauge */
  int mode;
};

struct ArtifactSpawnProperties
{
  /* Colour of artifacts to be spawned */
  float r; float g; float b;

  /* Random variation allowed in colour */
  float vr; float vg; float vb;

  /* Velocity of the artifacts */
  float dx; float dy; float dz;

  /* Random variation allowed in artifact velocity */
  float vdx; float vdy; float vdz;

  /* Growth rate and random variation in growth */
  float growth; float vgrowth;

  /* Decay and variation in decay */
  float decay; float vdecay;

  /* The next variables don't allow random variation as it seems somewhat
     senseless */

  /* Gravity of the artifacts */
  float ax; float ay; float az;

  /* Artifact source location */
  float x; float y; float z;

  /* Spawn size of the artifacts */
  float size;

  /* Life, i.e. beginning transparency value */
  float life;

  /* Frequency of the spawning, and number of particles spawned */
  int frequency;
  int spawned;

  /* Whether the particles have lighting */
  int lit;
};

struct TyreCollisionProperties *tyre_col;

/* Number of wheels in this scene */
int num_wheels;

/* Controller variables */
int gear;
int gear_changed;
float axis[3];
float d_axis[3];


int detach_trailers;
