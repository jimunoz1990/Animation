//Cloth simulation

#include "Cloth.h"
#include "FrameSaver.h"
#include "glut.h"
  
unsigned long lastUpdate;	// When was the last time we updated the cloth position
Cloth cloth(8,8,40,40);		// The cloth fabric were operating on

const int numBalls=2;
Vector3f ball_pos[numBalls]={Vector3f(3, -4,3.5),Vector3f(1, -4,3.5)};
float ball_radius[numBalls]={2,1};

bool started = false;		// Wait until space bar has been pressed before dropping the cloth
int Recording = 0 ;
FrameSaver FrSaver ;

int Width = 800;
int Height = 800 ;

// Initialize the GL context with the following properties
void init(GLvoid)
{
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth(1.0f);
	
	// Display with depth cues to prevent the sphere or cloth from incorrectly covering eachother
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);

	// Used to light the bottom back side
	glEnable(GL_LIGHT0);
	GLfloat lightPos[4] = {1.0,-0.5,-0.2,0.0};
	glLightfv(GL_LIGHT0,GL_POSITION,(GLfloat *) &lightPos);

	// Lights the top left
	glEnable(GL_LIGHT1);
	GLfloat lightAmbient1[4] = {0.0,0.0,0.0,0.0};
	GLfloat lightPos1[4] = {-1.0,1.0,0,0.0};
	GLfloat lightDiffuse1[4] = {0.5,0.5,0.3,0.0};
	glLightfv(GL_LIGHT1,GL_POSITION,(GLfloat *) &lightPos1);
	glLightfv(GL_LIGHT1,GL_AMBIENT,(GLfloat *) &lightAmbient1);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,(GLfloat *) &lightDiffuse1);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
}

void idleFunc() {
	// If the cloth has been dropped
	if (started) {
		// Apply a wind force to the cloth and calculate the pull of neighbouring particles
		cloth.calculateForces(Vector3f(0.04, -0.02,0.02),ball_pos, ball_radius, numBalls);

		// Get the current time so we can calculate how much time since the last update
		// used when integrating the distance a particle has traveled
		unsigned long currentUpdate = glutGet(GLUT_ELAPSED_TIME);

		// Move the cloth particles forward in time by how much time passed since the last update
		//cloth.update((currentUpdate - lastUpdate)/100.0f);
		cloth.update(0.1);
		lastUpdate = currentUpdate;
	}
}

void display() {
	// Clear the screen with the blue color set in the init
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	

	// Move the scene in front of the camera then rotate it before doing any drawing
	glPushMatrix();
	glTranslatef(-4,3.5,-9.0f);
	glRotatef(25,0,1,0);
	cloth.draw();

	// Move the ball then draw
	for (int i=0; i<numBalls;i++){
		glPushMatrix();
		glTranslatef(ball_pos[i].getX(),ball_pos[i].getY(),ball_pos[i].getZ());
		glColor3f(0.8f,0.3f,0.2f);
		glutSolidSphere(ball_radius[i]-0.1,50,50);
		glPopMatrix();
	}

	glPushMatrix();
	glColor3f(0.9f, 0.4f, 0.2f);
	glTranslatef(0, -0.5, 0);
	//glScalef(100,100,100);
	glTranslatef(0, -57, 0);
	glutSolidCube(100);
	glPopMatrix();
	// Clear the transforms and rotations applied earlier
	glPopMatrix();
	glPopMatrix();

	// Display the new frame
	glutSwapBuffers();
	glutPostRedisplay();
	if(Recording == 1)
    FrSaver.DumpPPM(Width, Height);
}

void myReshape(int w, int h)
{
    Width = w;
    Height = h;
}

void keyboard( unsigned char key, int x, int y ) 
{
	switch ( key ) {
	
	case 'r':
		cloth.setRotation();
		break;
	case 'm':
    if( Recording == 1 )
    {
        printf("Frame recording disabled.\n") ;
        Recording = 0 ;
    }
    else
    {
        printf("Frame recording enabled.\n") ;
        Recording = 1  ;
    }
    FrSaver.Toggle(Width);
    break ;

	case 's':
    FrSaver.DumpPPM(Width,Height) ;
    break;

	// On ESC quit
	case 27:    
		exit ( 0 );
		break;  

	// On Space drop the cloth or pause its fall
	case 32:
		started = !started;
		lastUpdate = glutGet(GLUT_ELAPSED_TIME);
		break;
	}
}

void reshape(int w, int h)  
{
	// Adjust the viewport if the user resizes the window
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();  
	if (h==0)  
		gluPerspective(80,(float)w,1.0,5000.0);
	else
		gluPerspective (80,( float )w /( float )h,1.0,5000.0 );
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity(); 
}

   
int main(int argc, char** argv) {
	// Initialize a window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH ); 
	glutInitWindowSize(800, 600 ); 
	glutCreateWindow( "Cloth Test" );

	// Initialize the GL context with predefined values
	init();

	// Link the window to the rest of the code
	glutDisplayFunc(display);  
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idleFunc);

	// Begin the window's event polling
	glutMainLoop();
	return 0;
}