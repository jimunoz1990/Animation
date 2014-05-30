/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

buggy with suspension.
this also shows you how to use geom groups.

*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define LENGTH 0.9	// chassis length
#define WIDTH 0.6	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.3	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass
#define MudRad 0.05 // mud particle radius


// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[6];
static dJointID joint[4];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[5];
static dGeomID spheres[50];
static dGeomID ground_box;
static dGeomID ground_box2;
static dGeomID ground_left;
static dGeomID ground_right;
static dGeomID ground_top;
static dGeomID ground_bot;
static dGeomID mud;
static dGeomID stand;
static dBodyID mudP[50];


// things that the user controls

static dReal speed=0,steer=0, speed2=0;	// user commands



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box || o1 == ground_box2 || o1 == ground_left || o1 == ground_bot || o1 == ground_right || o1 == ground_top || o1 == stand); // TODO: add all the other planes
  if(g1==0)
	  for(int i=0; i<50;i++)
		  if(o1==spheres[i])
			g1=1;

  int g2 = (o2 == ground || o2 == ground_box || o2 == ground_box2 || o2 == ground_left || o2 == ground_bot || o2 == ground_right || o2 == ground_top || o2 == stand); // TODO: add all the other planes
  if(g2==0)
	  for(int i=0; i<50;i++)
	     if(o2==spheres[i])
			g2=1;
  if (!(g1 ^ g2)) return;


  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
	  contact[i].surface.bounce = 1.0;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
	  "\t's' to begin the animation\n"
	  "\t'z' to decrease speed.\n"
	  "\t',' to steer left.\n"
	  "\t'.' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}


// called when a key pressed

static void command (int cmd)
{
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case ',':
    steer -= 0.5;
    break;
  case '.':
    steer += 0.5;
    break;
  case ' ':
    speed = 0;
    steer = 0;
    break;
  case 's':case 'S':
	dBodyAddTorque(body[5],0,0.1,0);
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}


// simulation loop

static void simLoop (int pause)
{
  int i;
  if (!pause) {
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

	dJointSetHinge2Param (joint[1],dParamVel2,-speed);
    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

	dJointSetHinge2Param (joint[1],dParamVel,v);
    dJointSetHinge2Param (joint[1],dParamFMax,0.2);
    dJointSetHinge2Param (joint[1],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[1],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[1],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }

  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,1,1);
  for (i=1; i<=5; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
				       dBodyGetRotation(body[i]),0.05f,RADIUS);

  for (i=0;i<50;i++) dsDrawSphere (dBodyGetPosition(mudP[i]),
					  dBodyGetRotation(mudP[i]),MudRad);

  dVector3 ss;

  dGeomBoxGetLengths (ground_box,ss);
  dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);
  dGeomBoxGetLengths (ground_box2,ss);
  dsDrawBox (dGeomGetPosition(ground_box2),dGeomGetRotation(ground_box2),ss);
  dGeomBoxGetLengths (ground_left,ss);
  dsDrawBox (dGeomGetPosition(ground_left),dGeomGetRotation(ground_left),ss);
  dGeomBoxGetLengths (ground_right,ss);
  dsDrawBox (dGeomGetPosition(ground_right),dGeomGetRotation(ground_right),ss);
  dGeomBoxGetLengths (ground_top,ss);
  dsDrawBox (dGeomGetPosition(ground_top),dGeomGetRotation(ground_top),ss);
  dGeomBoxGetLengths (ground_bot,ss);
  dsDrawBox (dGeomGetPosition(ground_bot),dGeomGetRotation(ground_bot),ss);
  dGeomBoxGetLengths (stand,ss);
  dsDrawBox (dGeomGetPosition(stand),dGeomGetRotation(stand),ss);
  dsSetColor (0.5,0.3,0.3);
  dGeomBoxGetLengths (mud,ss);
  dsDrawBox (dGeomGetPosition(mud),dGeomGetRotation(mud),ss);


  /*
  printf ("%.10f %.10f %.10f %.10f\n",
	  dJointGetHingeAngle (joint[1]),
	  dJointGetHingeAngle (joint[2]),
	  dJointGetHingeAngleRate (joint[1]),
	  dJointGetHingeAngleRate (joint[2]));
  */
}


int main (int argc, char **argv)
{
  int i;
  dMass m;

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  ground = dCreatePlane (space,0,0,1,-.05);

  // chassis body
  body[0] = dBodyCreate (world);
  dBodySetPosition (body[0],0,0,STARTZ);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[0],&m);
  box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);
  
  // wheel bodies
  for (i=1; i<=4; i++) {
    body[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (body[i],q);
    dMassSetSphere (&m,1,RADIUS);
    dMassAdjust (&m,WMASS);
    dBodySetMass (body[i],&m);
    sphere[i-1] = dCreateSphere (0,RADIUS);
    dGeomSetBody (sphere[i-1],body[i]);
  }
  dBodySetPosition (body[1],0.5*LENGTH,WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[2],0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[3],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
  dBodySetPosition (body[4],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

  // front wheel hinge
  /*
  joint[0] = dJointCreateHinge2 (world,0);
  dJointAttach (joint[0],body[0],body[1]);
  const dReal *a = dBodyGetPosition (body[1]);
  dJointSetHinge2Anchor (joint[0],a[0],a[1],a[2]);
  dJointSetHinge2Axis1 (joint[0],0,0,1);
  dJointSetHinge2Axis2 (joint[0],0,1,0);
  */

  // front and back wheel hinges
  for (i=0; i<4; i++) {
    joint[i] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[i],body[0],body[i+1]);
    const dReal *a = dBodyGetPosition (body[i+1]);
    dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[i],0,0,1);
    dJointSetHinge2Axis2 (joint[i],0,1,0);
  }

  // set joint suspension
  for (i=0; i<4; i++) {
    dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
  }

  // lock back wheels along the steering axis
  for (i=0; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
    // the following alternative method is no good as the wheels may get out
    // of alignment:
    //   dJointSetHinge2Param (joint[i],dParamVel,0);
    //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
  }

  //create isolated wheel for animations
 
  body[5] = dBodyCreate (world);
  dQuaternion q;
  dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
  dBodySetQuaternion (body[5],q);
  dMassSetSphere (&m,1,RADIUS);
  dMassAdjust (&m,WMASS);
  dBodySetMass (body[5],&m);
  sphere[4] = dCreateSphere (0,RADIUS);
  dGeomSetBody (sphere[4],body[5]);

  dBodySetPosition (body[5],-3*LENGTH,-WIDTH*5,HEIGHT*6);

  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);
  dSpaceAdd (car_space,box[0]);
  dSpaceAdd (car_space,sphere[0]);
  dSpaceAdd (car_space,sphere[1]);
  dSpaceAdd (car_space,sphere[2]);
  dSpaceAdd (car_space,sphere[3]);
  dSpaceAdd (car_space,sphere[4]);


  // environment
  ground_box = dCreateBox (space,0,0,0);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,2,0,-0.34);
  dGeomSetRotation (ground_box,R);

  ground_box2 = dCreateBox (space,0,0,0);
  dMatrix3 R2;
  dRFromAxisAndAngle (R2,0,1,0,0.15);
  dGeomSetPosition (ground_box2,3.8,0,-0.34);
  dGeomSetRotation (ground_box2,R2);
  
  // Making the mud hole
  ground_bot = dCreateBox (space,200,200,1);
  dGeomSetPosition (ground_bot,-98,0,-0.498);

  ground_right = dCreateBox (space,2,200,1);
  dGeomSetPosition (ground_right, 2.995,-102,-0.498);

  ground_left = dCreateBox (space,2,200,1);
  dGeomSetPosition (ground_left, 2.995,99,-0.498);

  ground_top = dCreateBox (space,200,200,1);
  dGeomSetPosition (ground_top,103.99,0,-0.498);

  mud = dCreateBox (space,2,1,1);
  dGeomSetPosition (mud,2.995,-1.5,-0.498);

  stand = dCreateBox (space,1,1,2.5);
  dGeomSetPosition (stand,-3*LENGTH,-WIDTH*5,-0.498);
  
  //create the particles that will be simulated as mud

  for (i=0; i<50; i++) {
	mudP[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (mudP[i],q);
    dMassSetSphere (&m,1,MudRad);
    dMassAdjust (&m,WMASS/100);
    dBodySetMass (mudP[i],&m);
    spheres[i] = dCreateSphere (0,MudRad);
    dGeomSetBody (spheres[i],mudP[i]);
  }

  //FIX THIS

	for (int i=0; i <10;i++)
		for(int j=0; j<5;j++)
		{
			dBodySetPosition (mudP[i*5+j],0+i/25.0,0+j/10.0,2);
			dGeomSetPosition (spheres[i*5+j],0+i/25.0,0+j/10.0,2);
			dSpaceAdd (car_space,spheres[i*5+j]);
		}
  
  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dGeomDestroy (sphere[2]);
  dGeomDestroy (sphere[3]);
  dGeomDestroy (sphere[4]);
  for (i=0; i<50; i++)  dGeomDestroy (spheres[i]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
