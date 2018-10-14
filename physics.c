/*
 * Third Year Project
 * Matt Kimber
 *
 * This file contains all the physics routines that an object can be 
 * loaded with.
 *
 */

/* Physics code */

/* Only include the graphics header file if it hasn't been included already */
#ifndef GRAPHICS
#include "graphics.h"
#endif

/* Same with the physics header */
#ifndef PHYSICS
#include "physics.h"
#endif

#ifndef CONFIG
#include "config.h"
#endif

/* System header files */
#include <stdio.h>

/* System timer functions */
#include <sys/timeb.h>

/* Wind drift for smoke */
#define WIND_X 0.000001
#define WIND_Z 0.0

#define DUST_MAX_THRESHOLD 0.9
#define DUST_MIN_THRESHOLD 0.04
#define SPARK_THRESHOLD 0.1

#define RADS_TO_RPM 9.52381

/* Time step */
const float h = (0.005 * PHYSICS_RATE);


/* Matrix functions */
void MatrixMultiply(dMatrix3, dMatrix3, dMatrix3);

void PHGenerateDust(float, float, float, float);
void PHGenerateSparks(float, float, float, float);

/* Sound functions */
void SetSkidVolume(float);

/* Graphics functions */
void GFXSpawnArtifacts(const struct ArtifactSpawnProperties *, int);


int p1;
int p2;

unsigned int frames;
unsigned int start_time;
long physics_ticks;

float fps;



/* Collision callback function */
void PHCollisionCallback(void *, dGeomID, dGeomID);

/* Code to get the number of milliseconds passed */
unsigned int GetMilliSeconds()
{
  struct timeb tb;
  ftime(&tb);
  return tb.time * 1000 + tb.millitm;
}

void PHPreStepWorld()
{
  dSpaceCollide(game_space,0, &PHCollisionCallback);

}

/* Step the ODE world */
void PHStepWorld()
{
  dWorldStep(game_world, h);

  /* Remove contact joints */
  dJointGroupEmpty(contact_group);
}

/* Callback for dSpaceCollide to check whether 2 objects collide */
void PHCollisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  /* Stolen from the example code */
  int i, n;
  int is_wheel = 0;
  int wheel_num;

  dJointID c;
  const int N = 2;
  dContact contact[N];


  for (i=0; i<num_wheels ; i++) {
    if ((o1 == tyre_col[i].body) || (o2 == tyre_col[i].body)) {
      is_wheel = 1;
      wheel_num = i;
    }
  }

  if (((o1 == car_body) || (o2 == car_body)) && is_wheel) return;

  n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if (n > 0) {
    for (i=0; i<n; i++) {

	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;

      /* These must change */
      if(is_wheel) {

	float load;
	contact[i].surface.mu = 0.0;
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.01;

	tyre_col[wheel_num].contact = 1;

	c = dJointCreateContact (game_world, contact_group, contact+i);
	dJointSetFeedback(c, &tyre_col[wheel_num].joint_info);
	dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));

	/* Get the load on the wheel */
	#define frc(a)  tyre_col[wheel_num].joint_info.f1[a]
	load = sqrt((frc(0) * frc(0)) + (frc(1) * frc(1)) + (frc(2) * frc(2)));

	tyre_col[wheel_num].load = load;
      } else {
	contact[i].surface.slip1 = 0.1;
	contact[i].surface.slip2 = 0.1;
	contact[i].surface.mu = 0.74;

	contact[i].surface.soft_erp = 0.5;
	contact[i].surface.soft_cfm = 0.1;

	/* If depth > threshold, create some dust */
	if((contact[i].geom.depth < DUST_MAX_THRESHOLD) &&
	(contact[i].geom.depth > DUST_MIN_THRESHOLD)){
	  PHGenerateDust(contact[i].geom.pos[0],
			 contact[i].geom.pos[1],
			 contact[i].geom.pos[2],
			 contact[i].geom.depth);


	  if (contact[i].geom.depth > SPARK_THRESHOLD) {
	    PHGenerateSparks(contact[i].geom.pos[0],
			   contact[i].geom.pos[1],
			   contact[i].geom.pos[2],
			   contact[i].geom.depth);
	  }
	}

	c = dJointCreateContact (game_world, contact_group, contact+i);
	dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));
      }

    }
  }


}

/* 
 * Generic object movement code 
 */
void PHGeneric(struct Object3D *obj)
{
  struct BasicProperties *prop;

  dReal *pos;
  dReal *r;
  int i;

  prop = (struct BasicProperties *)obj->physics_data;

  pos = (dReal *)dBodyGetPosition(prop->id);
  obj->x = pos[0];
  obj->y = pos[1];
  obj->z = pos[2];

  r = (dReal *)dBodyGetRotation(prop->id);
  for(i=0;i<12;i++) obj->r[i] = r[i];

}

void PHGenerateSmoke(const float x, const float y, const float z,
		     const float r, const float g, const float b,
		     const float size, const float growth, const float vgrowth,
		     const float decay)
{
  struct ArtifactSpawnProperties prop;

  prop.x = x; prop.y = y; prop.z = z;

  prop.dx = 0.0; prop.dy = -0.001; prop.dz = 0.0;
  prop.vdx = 0.003; prop.vdy = 0.0001; prop.vdz = 0.003;
  
  prop.ax = WIND_X; prop.ay = 0.000007; prop.az = WIND_Z;
  
  /* Grey-white colour - no variation */
  prop.r = r; prop.g = g; prop.b = b;
  prop.vr = 0.0; prop.vg = 0.0; prop.vb = 0.0;
  
  /* Start size and growth */
  prop.size = size; prop.growth=growth; prop.vgrowth=vgrowth;
  
  /* Start life and decay */
  prop.life = 0.1; prop.decay = decay; prop.vdecay = decay;

  /* Smoke *is* lit */
  prop.lit = 1;
  
  /* Add the artifacts */
  GFXSpawnArtifacts(&prop, 1);

  

}



void PHGenerateExhaustSmoke(struct CarProperties *prop)
{
  static float smoke_amount = 0.0;

  smoke_amount += ((axis[1] + 0.5) / 40.0);

  if(smoke_amount > 1.0) {
    dVector3 pos;

    dBodyGetRelPointPos(prop->id, prop->exhaust_pos[0], 
			prop->exhaust_pos[1], 
			prop->exhaust_pos[2], pos);

    smoke_amount = 0.0;
    PHGenerateSmoke(pos[0], pos[1], pos[2], 
		    0.9, 0.9, 1.0, 
		    0.15, 0.0015, 0.0001, 
		    0.0005);  
  }
}



void PHLights(float x, float y, float z, float r, float g, float b)
{

  struct ArtifactSpawnProperties prop;
  
  prop.x = x; prop.y = y; prop.z = z;
    
  prop.dx = 0.0; prop.dy = 0.0; prop.dz = 0.0;
  prop.vdx = 0.0; prop.vdy = 0.0; prop.vdz = 0.0;
  
  prop.ax = 0.0; prop.ay = 0.0; prop.az = 0.0;
  
  prop.r = r; prop.g = g; prop.b = b;
  prop.vr = 0.0; prop.vg = 0.0; prop.vb = 0.0;
  
  prop.size = 0.1; prop.growth=-0.0004; prop.vgrowth=0.0;
  prop.life = 0.1; prop.decay = 0.01; prop.vdecay = 0.0;
  prop.lit = 0;
  
  GFXSpawnArtifacts(&prop, 1);

}

void PHBrakeLights(struct CarProperties *cprop)
{

  dVector3 pos;
  int i;
  
  for(i=0;i<2;i++) {
    dBodyGetRelPointPos(cprop->id, cprop->brake_light_pos[i][0], 
			cprop->brake_light_pos[i][1], 
			cprop->brake_light_pos[i][2], pos);
  
    PHLights(pos[0], pos[1], pos[2], 1.0, 0.0, 0.0);

  }


}


void PHReverseLights(struct CarProperties *cprop)
{
  dVector3 pos;
  int i;  
  for(i=0;i<2;i++) {
    dBodyGetRelPointPos(cprop->id, cprop->reverse_light_pos[i][0], 
			cprop->reverse_light_pos[i][1], 
			cprop->reverse_light_pos[i][2], pos);
  
    PHLights(pos[0], pos[1], pos[2], 1.0, 1.0, 1.0);
  }
}

void PHIndicators(struct CarProperties *cprop, int direction)
{
  dVector3 pos;
  int i;

  if (physics_ticks % 250 < 125) return;

  for(i=0;i<2;i++) {
    dBodyGetRelPointPos(cprop->id, 
			cprop->indicator_pos[direction][i][0], 
			cprop->indicator_pos[direction][i][1], 
			cprop->indicator_pos[direction][i][2], pos);
  
    PHLights(pos[0], pos[1], pos[2], 1.0, 0.5, 0.0);
  }
}

void PHGenerateSparks(float x, float y, float z, float amount)
{
  int j;
  struct ArtifactSpawnProperties prop;
  
  j = (int)((amount + 1.0) * 1000.0);
  if (j>100) j = 100;

  prop.x = x; prop.y = y; prop.z = z;
  prop.dx = 0.0; prop.dy = -0.01; prop.dz = 0.0;
  prop.vdx = 0.1; prop.vdy = 0.1; prop.vdz = 0.1;
  prop.ax = WIND_X; prop.ay = -0.002; prop.az = WIND_Z;
  prop.r = 1.0; prop.g = 0.5; prop.b = 0.0;
  prop.vr = 0.3; prop.vg = 0.5; prop.vb = 0.0;
  prop.size = 0.03; prop.growth=0.0; prop.vgrowth=0.0;
  prop.life = 1.0; prop.decay = 0.01; prop.vdecay = 0.01;
  prop.lit = 0;
  
  /* Add the artifacts */
  GFXSpawnArtifacts(&prop, j);
}

void PHGenerateDust(float x, float y, float z, float amount)
{
  struct ArtifactSpawnProperties prop;
  
  prop.x = x; prop.y = y; prop.z = z;
  prop.dx = 0.0; prop.dy = -0.001; prop.dz = 0.0;
  prop.vdx = 0.003; prop.vdy = 0.002; prop.vdz = 0.003;
  prop.ax = WIND_X; prop.ay = 0.000007; prop.az = WIND_Z;
  prop.r = 0.8; prop.g = 0.6; prop.b = 0.1;
  prop.vr = 0.0; prop.vg = 0.0; prop.vb = 0.0;
  prop.size = 0.45; prop.growth=0.02; prop.vgrowth=0.002;
  prop.life = 0.25 * amount; prop.decay = 0.0005; prop.vdecay = 0.0;
  prop.lit = 1;
  
  /* Add the artifacts */
  GFXSpawnArtifacts(&prop, 1);
}

void PHGenerateWheelSmoke(float x, float y, float z, float amount, int wheel)
{
  static float smoke_amount[4] = {0.0, 0.0, 0.0, 0.0};

  if(wheel > 3) return;

  smoke_amount[wheel] += amount;

  if(smoke_amount[wheel] > 1.0) {
    smoke_amount[wheel] = 0.0;
    PHGenerateSmoke(x, y, z, 1.0, 1.0, 1.0, 0.45, 0.0025, 0.0001, 0.0001);
  }
  

}

float PHEngineTorque(const struct CarProperties *prop)
{

  float r, range, p1, p2;
  int i;

  range = (prop->engine.max_rpm - prop->engine.idle_rpm);
  r = (prop->engine.rpm * 9.0) / range;


  i = (int)r;

  p1 = (prop->engine.rpm - ((float)i * (range/9.0))) / 1000.0;
  p2 = 1.0 - p1;

  if (prop->engine.rpm > prop->engine.max_rpm) return 0.0;

  return (prop->engine.max_torque * 
	  ((p2 * prop->engine.torque_curve[i]) + 
	  (p1 * prop->engine.torque_curve[i+1])));
}



/* Drivetrain code */
void PHDrivetrain(struct CarProperties *prop)
{
  int i;

  int driven_wheels = 0;
  float total_w = 0, total_radius = 0;
  float new_velocity;

  float engine_torque;
  float clutch_amount;
  float feedback_torque = 0.0;
  float shaft_torque = 0.0;

  static float change_clutch;

  /* Restrictions on gear number */
  if (gear < 0) gear = 0;
  if (gear > (prop->gearbox.num_gears-1)) gear = (prop->gearbox.num_gears-1);

  /* Idle throttle amount */
  if(axis[1] < prop->engine.idle_throttle) 
    axis[1] = prop->engine.idle_throttle;

  /* Reduce the clutch feed amount */
  if(change_clutch > 0.0) {
    if (gear <= 2) { change_clutch -= (h/100.0); }
    else { change_clutch -= (h/10.0); }
  } else {
    change_clutch = 0.0;
  }

  /* If there is a gearchange, smoothly feed the clutch in */
  if(gear_changed) {
    gear_changed = 0;
    change_clutch = 1.0;
  }

  /* Sum torques on the drive wheels */
  for(i=0;i<4;i++) {

    if((axis[2] > 0.01) && (prop->wheel[i].w < 0.0)) {
      prop->wheel[i].torque += prop->wheel[i].brake_force * axis[2] * 0.002;
    } else if((axis[2] > 0.01) && (prop->wheel[i].w > 0.0)) {
      prop->wheel[i].torque -= prop->wheel[i].brake_force * axis[2] * 0.002;
    }


    if(prop->wheel[i].driven) {
      driven_wheels++;
      total_w+=prop->wheel[i].w;
      total_radius+=prop->wheel[i].radius;
    }

  }

  total_w = total_w * 5.0;

  /* The shaft rotates at the same speed as the wheels */
  prop->engine.shaft_rpm = (-total_w * RADS_TO_RPM) / driven_wheels;

  /* Disengage the clutch as we approach low RPM */
  clutch_amount = (prop->engine.rpm - 500.0) - (prop->engine.idle_rpm);
  clutch_amount = clutch_amount / 500.0;
  if (clutch_amount > 1.0) clutch_amount = 1.0;

  clutch_amount -= change_clutch;
  if (clutch_amount < 0.0) clutch_amount = 0.0;


  /* If we are in gear, lock the engine RPM */
  if (fabs(prop->gearbox.ratio[gear]) > 0.0) {
    float rpm_adjust;
    rpm_adjust = ((prop->engine.rpm - (prop->engine.shaft_rpm *
				       prop->gearbox.ratio[gear])) *
		  clutch_amount);

    prop->engine.rpm -= 0.2 * (rpm_adjust / prop->engine.inertia);
  }


  /* Get the torque the engine produces */
  engine_torque = PHEngineTorque(prop) * axis[1];

  /* Add the internal engine braking to the feedback torque */
  feedback_torque += ((prop->engine.rpm / 60.0) * 0.74);

  /* Work out the resultant torque from the engine */
  engine_torque -= feedback_torque;

  shaft_torque = (engine_torque * sqrt(clutch_amount)) * 
    prop->gearbox.ratio[gear];
 
  /* Affect the engine RPM */
  prop->engine.rpm += (engine_torque / prop->engine.inertia);


  for(i=0;i<4;i++) {
    if(prop->wheel[i].driven) {


      /* 0.9 = transmission efficiency */
      prop->wheel[i].torque -= ((shaft_torque * 0.7) / 
				(driven_wheels * 500.0) );
    }




  }

  /* Inform the game of the property values */
  car_rpm = prop->engine.rpm;
  new_velocity = -((total_w / driven_wheels) *
		  (total_radius / driven_wheels));

  car_velocity = ((car_velocity * 9.0) + (new_velocity)) / 10.0;

}

/* The tyre model */
void PHTyreModel(struct WheelProperties *wheel)
{
  /*** TYRE MODEL PARAMETERS ***/
  const dReal *linear_vel;
  float load;
  float slide_amount;
  dVector3 r, body_r;
  dVector3 force_wrt_wheel;
  int j;
  static float skid_vol = 0.0;
  /*** END TYRE MODEL PARAMETERS ***/



  /*** START OF TYRE MODEL ***/
  /* Get the linear velocity of the wheel */
  linear_vel = dBodyGetLinearVel(wheel->id);

  /* Get the direction the wheel is heading in */

  
  /* Get a unit vector for the heading of the wheel */   
  dBodyVectorToWorld(wheel->id, 0.0, 0.0, 1.0, r);


  wheel->w += h * (wheel->torque / ((wheel->mass / 100.0) * 
				    (wheel->radius * wheel->radius)));
  wheel->torque = 0.0;

  
  
  for(j=0;j<3;j++) {
    body_r[j] = r[j] * wheel->w * wheel->radius;
  }
  
  
  if (tyre_col[wheel->col].contact) {

    dVector3 cp_vel, body_vel;
    float slip_angle, slip_ratio;
    float lat, lng, len, max;
    
    /* Get the load on the wheel */
    load = tyre_col[wheel->col].load;
    
    /* Get the contact patch velocity wrt the tyre from the
       contact patch velocity wrt the world */
    dBodyVectorFromWorld(wheel->id,
			 body_r[0] - linear_vel[0],
			 body_r[1] - linear_vel[1],
			 body_r[2] - linear_vel[2],
			 cp_vel);
    
    dBodyVectorFromWorld(wheel->id,
			 linear_vel[0],
			 linear_vel[1],
			 linear_vel[2],
			 body_vel);
    

    /* Slip ratio calculation  */
    slip_ratio = (sqrt((cp_vel[0] * cp_vel[0]) + (cp_vel[2] * cp_vel[2])) / 
	     sqrt((body_vel[0] * body_vel[0]) + (body_vel[2] * body_vel[2])))
      + 1.0;

    /* 
     * This is from an SAE paper - when the longitudinal slip is
     * small, we should use a first-order differential eqn. to
     * calculate the slip angle and not a simple arctangent.
     *
     * The damping is necessary because otherwise we get
     * oscillation.
     *
     * Due to Erik M. Lowndes and Ruud van Gaal
     */

    slip_angle = atan2f(cp_vel[0], fabs(cp_vel[2]));


    if(fabs(cp_vel[2]) < 0.5 ) {
      wheel->tanSA -= h * ((fabs(cp_vel[2])) * wheel->tanSA) +
	cp_vel[0];
      slip_angle = atanf(-wheel->tanSA);
    
      max = fabs(cp_vel[2]);
      if (fabs(cp_vel[0]) < max) max = fabs(cp_vel[0]);
      
      slip_angle = slip_angle * max * 0.5;

    }
  


    wheel->tanSA = tanf(slip_angle);

    /* Use Gregor Veble's combined pacejka formula to factor down
       the slip angle and ratio to account for loss of lateral
       traction during a wheel spin or slide */
    lat = wheel->tanSA / tanf(wheel->optSA);
    lng = slip_ratio / wheel->optSR;

    len = sqrt((lat*lat) + (lng*lng));

    slip_angle = len * wheel->optSA;
    slip_ratio = len * wheel->optSR;
    

    if(load > 20.0) {
      /* Damage the wheel */
      wheel->damage += (load - 20.0);
      /*
       * printf("Applying %f of damage to wheel %d - total damage %f\n", 
       *	     (load - 10.0), wheel->col, wheel->damage);
       */
    }

    if(fabs(cp_vel[2]) < 0.1) slide_amount = 0.0;
    else {
      slide_amount = ((cp_vel[2]*cp_vel[2]) + (cp_vel[0] * cp_vel[0]));
      if(slide_amount > 2.5) slide_amount = 2.5;
    }

    /* If slip angle or ratio are high, then generate smoke */ 
    if(slide_amount > 1.0) {
      
      if ((slide_amount - 1.0) > skid_vol) skid_vol = (slide_amount - 1.0);
      
      if(slide_amount > 2.0) {
	dReal *pos;
	
	pos = (dReal *)dBodyGetPosition(wheel->id);
	
	/* Call the smoke generation function */
	PHGenerateWheelSmoke(pos[0], pos[1], pos[2], 
			     ((slide_amount - 2.0) * 0.1), 
				     wheel->col);
      }
    } 
    
    if (physics_ticks % 100 == 0) {
      SetSkidVolume(skid_vol);
      skid_vol = skid_vol / 2;
    }
    
    /* Trim load to stop random explosions when dropping car*/
    if(load > 100.0) load = 100.0;
    if(load < -100.0) load = -100.0;

    /* Zero force at zero slip */
    if (fabs(cp_vel[0]) < 0.00000001) {
      force_wrt_wheel[0] = 0.0;
    } else {
      force_wrt_wheel[0] = SGN(cp_vel[0]) * load * (wheel->lat.B * slip_angle) 
	/  (1.0 + pow(fabs(wheel->lat.A * slip_angle), wheel->lat.P));
    }

    if (fabs(cp_vel[2]) < 0.00000001) {
      force_wrt_wheel[2] = 0.0;
    } else {
      force_wrt_wheel[2] = SGN(cp_vel[2]) * load * (wheel->lng.B * slip_ratio) 
	/ (1.0 + pow(fabs(wheel->lng.A * slip_ratio), wheel->lng.P));
    }

    
    wheel->torque = -force_wrt_wheel[2] * wheel->radius;
    
    /* Do nothing with the vertical direction */
    force_wrt_wheel[1] = 0.0;
    
    /* 
     * Translate the forces from the wheel co-ordinate system to the world 
     */              
    dBodyAddRelForceAtRelPos(wheel->id, 
			  force_wrt_wheel[0], 
			  force_wrt_wheel[1], force_wrt_wheel[2],
			  0.0, -wheel->radius, 0.0);

    tyre_col[wheel->col].contact = 0;    
    /*** END OF TYRE MODEL ***/
    
  }
}

/* Code for a car */
void PHCar(struct Object3D *obj)
{


  struct CarProperties *prop;
  
  dReal *pos;
  dReal *r;

  int i,j;

  prop = (struct CarProperties *)obj->physics_data;

  pos = (dReal *)dBodyGetPosition(prop->id);
  obj->x = pos[0];
  obj->y = pos[1];
  obj->z = pos[2];

  r = (dReal *)dBodyGetRotation(prop->id);
  for(i=0;i<12;i++) obj->r[i] = r[i]; 


  PHDrivetrain(prop);


  for(i=0;i<=3;i++) {    
    dReal *rot;
    dMatrix3 M,M1,M2;
  
    prop->wheel[i].ry = axis[0] * prop->wheel[i].s_lock;


    /* Rotate the wheel according to the steering */      
    
    rot = (dReal *)dBodyGetRotation(prop->wheel[i].id);
    for(j=0;j<12;j++) M1[j] = rot[j];
    dRFromAxisAndAngle(M2, 0.0, 1.0, 0.0, prop->wheel[i].ry);
    MatrixMultiply(M1,M2,M);
    
    dBodySetRotation(prop->wheel[i].id, M);

   
    /* Call the tyre model */
    if (! prop->wheel[i].broken ) PHTyreModel(&prop->wheel[i]);

    
    /* Do the damage routine */
    //    if ((!prop->wheel[i].broken) && (prop->wheel[i].damage > 50.0)) {
      /* Break the suspension */
      /* printf("Breaking wheel %d\n", i); */
    //      dJointAttach(prop->susp[i], 0, 0);
    //      tyre_col[prop->wheel[i].col].body = 0;
    //      prop->wheel[i].broken = 1;
    //    }

    /* Reset the body rotation */
    dBodySetRotation(prop->wheel[i].id, M1);

    prop->wheel[i].rx += prop->wheel[i].w / 100.0;

    tyre_col[prop->wheel[i].col].contact = 0;
  }

    /* Generate the exhaust smoke */
    PHGenerateExhaustSmoke(prop);

    if (axis[2] > 0.01) PHBrakeLights(prop);
    if (prop->gearbox.ratio[gear] < 0.0) PHReverseLights(prop);
    if (axis[0] < -0.9) PHIndicators(prop, 1);
    if (axis[0] > 0.9) PHIndicators(prop, 0);

}
                                       

/* 2 wheeled no-suspension basic trailer */
void PHTrailer2Wheel(struct Object3D *obj)
{
  struct Trailer2WheelProperties *prop;

  dReal *pos;
  dReal *r;
  int i;

  prop = (struct Trailer2WheelProperties *)obj->physics_data;

  pos = (dReal *)dBodyGetPosition(prop->id);
  obj->x = pos[0];
  obj->y = pos[1];
  obj->z = pos[2];

  r = (dReal *)dBodyGetRotation(prop->id);
  for(i=0;i<12;i++) obj->r[i] = r[i]; 

  if((detach_trailers) && (prop->attached)) {
    dJointAttach(prop->joint,0,0);
  }

  /* Apply the brakes */
  
   for(i=0;i<2;i++) {

     if(prop->attached) {
       if((axis[2] > 0.01) && (prop->wheel[i].w < 0.0)) {
	 prop->wheel[i].torque += prop->wheel[i].brake_force * axis[2] * 0.01;
       } else if((axis[2] > 0.01) && (prop->wheel[i].w > 0.0)) {
	 prop->wheel[i].torque -= prop->wheel[i].brake_force * axis[2] * 0.01;
       }
     }
  

    /* Call the tyre model */
    if (! prop->wheel[i].broken ) PHTyreModel(&prop->wheel[i]);

    
    /* Do the damage routine */
    //    if ((!prop->wheel[i].broken) && (prop->wheel[i].damage > 30.0)) {
      /* Break the suspension */
      /* printf("Breaking wheel %d\n", i); */
    //      dJointAttach(prop->susp[i], 0, 0);
    //      tyre_col[prop->wheel[i].col].body = 0;
    //      prop->wheel[i].broken = 1;
    //    }

    prop->wheel[i].rx += prop->wheel[i].w / 100.0;
    tyre_col[prop->wheel[i].col].contact = 0;
  }
  
  
}

/* Code for a location-based trigger */
void PHTrigger(struct Object3D *obj)
{
  
 
}

/* Code for the graphical display bit of a wheel */
void PHWheel(struct Object3D *obj)
{
  /* Set the wheel position to the car */
  struct WheelRepProperties *prop;

  dReal *pos;
  dReal *r;
  int i;
  dMatrix3 rm1, rm2, om, M;

  prop = (struct WheelRepProperties *)obj->physics_data;

  pos = (dReal *)dBodyGetPosition(prop->wheel->id);
  obj->x = pos[0];
  obj->y = pos[1];
  obj->z = pos[2];
  
  dRFromAxisAndAngle(rm2, 0.0, 1.0, 0.0, prop->wheel->ry);
  dRFromAxisAndAngle(rm1, 1.0, 0.0, 0.0, prop->wheel->rx);

  r = (dReal *)dBodyGetRotation(prop->wheel->id);
  for(i=0;i<12;i++) om[i] = r[i]; 
  
  MatrixMultiply(om, rm2, M);
  MatrixMultiply(M, rm1, obj->r);
 
}

void PHFollow(struct Object3D *obj)
{
  struct FollowerProperties *prop;

  dReal *pos;
  dReal *r;
  int i;

  prop = (struct FollowerProperties *)obj->physics_data;

  pos = (dReal *)dBodyGetPosition(prop->object);
  obj->x = pos[0];
  obj->y = pos[1];
  obj->z = pos[2];

  /*  printf("%f %f %f\n", obj->x, obj->y, obj->z); */
  r = (dReal *)dBodyGetRotation(prop->object);
  for(i=0;i<12;i++) obj->r[i] = r[i]; 

}

/* Physics model for car gauges */
void PHGauge(struct Object3D *obj)
{
  /* 
   * Speedometer.  Rotation of the object is determined by the speed of
   * the car being tracked.
   */
  
  float val, rot;
  struct GaugeProperties *prop;

  prop = (struct GaugeProperties *)obj->physics_data;

  if (prop->mode == 0) { val = car_rpm; }
  else { val = car_velocity; }


  if (val > prop->max) val = prop->max;
  if (val < prop->min) val = prop->min;

  rot = (((val - prop->min) / (prop->max - prop->min)) * 
     (prop->max_rot - prop->min_rot)) + prop->min_rot;

  dRFromAxisAndAngle(obj->r, 0.0, 0.0, 1.0, rot);
  
}

/* Null physics - object that does nothing */
void PHNothing(struct Object3D *obj)
{
  /* Do nothing */

}


void UpdateAxes()
{
#ifndef JOYSTICK
  int i;

  
  for(i=0;i<3;i++) {
    /* Update the controller axes */
    axis[i] += d_axis[i];

    /* Limit axis */
    if (axis[i] > 1.0) axis[i] = 1.0;
    else if (axis[i] < -1.0) axis[i] = -1.0;
 
    /* Return to zero over time */
    if (axis[i] > 0.0025) axis[i] -= 0.0025; 
    else if (axis[i] < -0.0025)  axis[i] += 0.0025;
    else axis[i] = 0;
  }

#endif

  if(axis[1] < 0.0) axis[1] = 0.0;
  if(axis[2] < 0.0) axis[2] = 0.0;

}

/* 
 * Code for the artifact spawning object 
 */
void PHArtifactSpawn(struct Object3D *obj)
{
  struct ArtifactSpawnProperties *prop;

  prop = (struct ArtifactSpawnProperties *)obj->physics_data;

  if (physics_ticks % prop->frequency == 0) {
    GFXSpawnArtifacts(prop, prop->spawned);
  }

}

/*
 * Code for the various lighting adjustment functions
 */

void PHLightFlicker(struct Light3D *light)
{
  int i;
  float f;
  
  /* If the correct number of physics ticks has passed, change
     the light colour */
  if(physics_ticks - light->last_flickered > light->flicker) {
    light->last_flickered = physics_ticks;

    for(i=0;i<4;i++) {
      f = light->diffuse[i];
      light->diffuse[i] = light->flicker_colour[i];
      light->flicker_colour[i] = f;
    }

  }
}

void PHLightCarMount(struct Light3D *light)
{
  /* Make the light follow the car around */
  struct CarProperties *prop;

  prop = (struct CarProperties *)light->car;

  dBodyGetRelPointPos(prop->id, light->pos[0], light->pos[1], 
  		      light->pos[2], light->position);

  
}

/* If the physics are up-to-date, we can draw a graphics frame */
int PHShouldDrawGraphics()
{
  int ticks;
  static int not_drawn_for = 0;

  ticks = (GetMilliSeconds() - start_time) - (physics_ticks * PHYSICS_RATE);

  if (ticks > 0) {
    not_drawn_for++;
    if(not_drawn_for < 50) {
      return 0;
    } else {
      not_drawn_for = 0;
      return 1;
    }
  }

  else return 1;


}

void UpdateTimingInfo()
{
  if (start_time == 0) {
    start_time = GetMilliSeconds();
    frames = 0;
  }

  physics_ticks++;

}
