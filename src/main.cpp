/*
 * Program 3 base code - includes modifications to shape and initGeom in preparation to load
 * multi shape objects 
 * CPE 471 Cal Poly Z. Wood + S. Sueda + I. Dunn
 */

#include <iostream>
#include <glad/glad.h>

#include "GLSL.h"
#include "Program.h"
#include "Shape.h"
#include "MatrixStack.h"
#include "WindowManager.h"
#include "Texture.h"
#include "Spline.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader/tiny_obj_loader.h>

// value_ptr for glm
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define PARTICLE_SIZE 0.045


#define X 0
#define Y 1
#define Z 2

using namespace std;
using namespace glm;

const float TO_RAD = M_PI / 180;

double get_last_elapsed_time()
{
	static double lasttime = glfwGetTime();
	double actualtime =glfwGetTime();
	double difference = actualtime- lasttime;
	lasttime = actualtime;
	return difference;
}

enum BodyType {
	STATIONARY = 0,
	MOVING = 1
};

class Body {
	public:
		int mass;
		vec3 velocity;
		vec3 rotational_velocity;
		vec3 rotation;
		vec3 center_of_mass;

		GLuint particlesID;
		GLfloat *particles;
		BodyType type;
		vec3 getNewPos(vec3 new_rot, vec3 velocity_transform, vec3 last_pos) {
			vec3 center_vec = last_pos - center_of_mass;
			//cout << center_vec.x << ", " << center_vec.y << ", " << center_vec.z << endl;

			//mat4 Initial_Trans = glm::translate(glm::mat4(1.0f), -last_pos);
			mat4 RotX = glm::rotate( glm::mat4(1.0f), new_rot.x * TO_RAD, vec3(1, 0, 0));
			mat4 RotY = glm::rotate( glm::mat4(1.0f), new_rot.y * TO_RAD, vec3(0, 1, 0));
			mat4 RotZ = glm::rotate ( glm::mat4(1.0f), new_rot.z * TO_RAD, vec3(0, 0, 1));
			//mat4 Final_Trans = glm::translate(glm::mat4(1), last_pos);

			mat4 RotationMatrix = RotX * RotY * RotZ;
			vec4 rotation_vec = vec4(center_vec, 1.0f) * RotationMatrix;
			return center_of_mass + vec3(rotation_vec) + velocity_transform;
		}

		void updatePos(float frametime) {
			vec3 new_rotation = rotational_velocity * frametime;
			vec3 velocity_transform = velocity * frametime;

			rotation += new_rotation;
			// for every particle
			for (int i = 0; i < mass * 3; i += 3) {

				vec3 last_pos = vec3(particles[i + X], particles[i + Y], particles[i + Z]);
				
				
				
				// get rotation about the center_of_mass

				vec3 new_pos = getNewPos(new_rotation, velocity_transform, last_pos);
				
				// update particle X, Y, Z based on how much time went by
				// particles[i + X] += velocity.x * frametime;
				// particles[i + Y] += velocity.y * frametime;
				// particles[i + Z] += velocity.z * frametime;

				particles[i + X] = new_pos.x;
				particles[i + Y] = new_pos.y;
				particles[i + Z] = new_pos.z;
			}

			center_of_mass += velocity_transform;
		}
		void checkCollision(Body b2) {
			// if (type == STATIONARY) {
			// 	cout << "Error: checkCollision called on a Stationary body" << endl;
			// 	exit(1);
			// }
			for (int i = 0; i < mass; i++) {
				for (int j = 0; j < b2.mass; j++) {
					vec3 p = vec3(particles[i*3 + X], particles[i*3 + Y], particles[i*3 + Z]);
					vec3 p2 = vec3(b2.particles[j*3 + X], b2.particles[j*3 + Y], b2.particles[j*3 + Z]);

					float dist = distance(p, p2);
					if (dist <= PARTICLE_SIZE) {
						
						// normal vector pointed from p2 to p
						vec3 normal = normalize(p - p2);
						
						if (b2.type == MOVING) {

							vec3 relativeVelocity = velocity - b2.velocity;
							float dot_product = dot(relativeVelocity, normal);
							velocity -= dot_product * normal;
							b2.velocity += dot_product * normal;

						} else {
			// 				// TODO: check the physics here

							//velocity -= 2 * dot(velocity, normal) * normal;
							


							vec3 relativeVelocity = velocity - b2.velocity;
							float parallel_velocity_of_collision = dot(relativeVelocity, normal);
							vec3 orthagonal_velocity_of_collision = cross(relativeVelocity, normal);

							// calculate change in velocity
							velocity -= 2 * parallel_velocity_of_collision * normal;

							// calculate change in rotational velocity
							rotational_velocity = -1000.0f * orthagonal_velocity_of_collision;
							cout << "rotational_velocity = " << orthagonal_velocity_of_collision.x << ", " << orthagonal_velocity_of_collision.y << ", " << orthagonal_velocity_of_collision.z << endl;

						}

						//set this particle outside the bounds of particle p (no duplicate collisions)
						vec3 offset = 1.05f * (float)(PARTICLE_SIZE - abs(dist)) * normal;
						center_of_mass += offset;
						for (int t = 0; t < mass * 3; t += 3) {
							particles[t + X] += offset.x;
							particles[t + Y] += offset.y;
							particles[t + Z] += offset.z;
						}
					}

				}
			}
		}

};


class camera
{
public:
	glm::vec3 pos;
	float pitch = 0;
	float yaw = -90;
	float rotAngle;
	int w, a, s, d;
	mat4 cameraMatrix = mat4(1.0f);
	camera()
	{
		w = a = s = d = 0;
		rotAngle = 0.0;
		pos = glm::vec3(0, 0, 0);
	}
	glm::mat4 process(double ftime)
	{
		vec3 dir;
		dir.x = cos(yaw*TO_RAD) * cos (pitch*TO_RAD);
		dir.y = -1.0 * sin(pitch*TO_RAD);
		dir.z = sin(yaw*TO_RAD) * cos(pitch*TO_RAD);

		vec3 front = normalize(dir);
		vec3 up = vec3(0,1,0);
		vec3 side = cross(front, up);

		float forward_speed = 0;
		float side_speed = 0;
		float velocity = 0.8;
		if (w == 1)
		{
			forward_speed = velocity*ftime;
		}
		else if (s == 1)
		{
			forward_speed = -velocity*ftime;
		}
		float yangle=0;
		if (a == 1)
			side_speed = -velocity*ftime;
		else if(d==1)
			side_speed = velocity*ftime;


		

		// get the rotation matrix
		mat4 R = lookAt(vec3(0), front, up);
		
		pos += -forward_speed * front + -side_speed * side;
		glm::mat4 T = glm::translate(glm::mat4(1), pos);
		cameraMatrix = R*T;
		return cameraMatrix;
	}
};

camera mycam;

/*class Particle {
	public:
		vec3 pos;
		vec3 velocity;
		vector<Particle> bonds;
		vec3 bondVelocity = vec3(0);
		Particle(vec3 init_pos, vec3 init_v) {
			pos = init_pos;
			velocity = init_v;
		}
		void checkCollision(Particle &p) {
			if (type == STATIONARY) {
				return;
			}

			float dist = distance(pos, p.pos);
			if (dist <= PARTICLE_SIZE) {
				
				vec3 normal = normalize(pos - p.pos);
				
				if (p.type == MOVING) {

					vec3 relativeVelocity = velocity - p.velocity;
					float dot_product = dot(relativeVelocity, normal);
					velocity -= dot_product * normal;
					p.velocity += dot_product * normal;

				} else {
					// TODO: check the physics here

					velocity -= 2 * dot(velocity, normal) * normal; 
				}

				// set this particle outside the bounds of particle p (no duplicate collisions)
				pos += (float)(PARTICLE_SIZE - abs(dist)) * normal;
			}
		}
		void addBond(Particle &p) {
			bonds.push_back(p);
		}
		void updatePos(float elapsedTime) {
			//pos += elapsedTime * (velocity + bondVelocity);
			pos += elapsedTime * velocity;

			// for (int i = 0; i < bonds.size(); i++) {
			// 	float dist = distance(pos, bonds[i].pos);
			// 	if (dist > PARTICLE_SIZE * 1.01) {
			// 		bondVelocity += (float)dist * normalize(bonds[i].pos - pos);
			// 	} else {
			// 		bondVelocity = vec3(0);
			// 	}
			// }
			
		}
};*/

class BoundingBox
{
	public:
		vec3 min;
		vec3 max;
		BoundingBox(vec3 mn, vec3 mx) {
			min = mn;
			max = mx;
		}
};

class Instance 
{
	public:
		vec3 pos;
		vec3 rot;
		vec3 scale;
		Instance(vec3 p, vec3 r, vec3 s) {
			pos = p;
			rot = r;
			scale = s;
		}
};

class CompoundShape
{
	public:
		vec3 min;
		vec3 max;
		float height = 0;
		float width = 0;
		float depth = 0;
		mat4 transform = mat4(1.0f);
		vector<shared_ptr<Shape>> shapes;
		vector<Instance> instances;
		

		CompoundShape() {

			min = vec3(std::numeric_limits<float>::max());
			max = vec3(std::numeric_limits<float>::min());

		}
		void load(string filename) {
			vector<tinyobj::shape_t> shapeObjects;
			vector<tinyobj::material_t> objMaterials;
			string errStr;

			//load in the mesh and make the shape(s)
			bool rc = tinyobj::LoadObj(shapeObjects, objMaterials, errStr, (filename).c_str());

			if (!rc) {
				cerr << errStr << endl;
			} else {
				//for now all our shapes will not have textures - change in later labs
				for (int i = 0; i < shapeObjects.size(); i++) {
					addShape(shapeObjects[i]);
				}
			}
			height = max.y - min.y;
			width = max.x - min.x;
			depth = max.z - min.z;
		}
		void addShape(tinyobj::shape_t shape_obj) {
			shared_ptr<Shape> shape = make_shared<Shape>();
			shape->createShape(shape_obj);
			
			shape->measure();
			shape->init();

			min = vec3(std::min(min.x, shape->min.x), std::min(min.y, shape->min.y), std::min(min.z, shape->min.z));
			max = vec3(std::max(max.x, shape->max.x), std::max(max.y, shape->max.y), std::max(max.z, shape->max.z));

			shapes.push_back(shape);
		}
		void addInstance(vec3 pos, vec3 rot, vec3 scale) {
			instances.push_back(Instance(pos, rot, scale));
		}
		void removeInstance(int i) {
			instances.erase(instances.begin()+i);
		}
		void setModel(shared_ptr<Program> prog, Instance instance) {
			setModel(prog, instance.pos, instance.rot.x, instance.rot.y, instance.rot.z, instance.scale);

		}
		void setModel(shared_ptr<Program> prog, vec3 trans, float rotX, float rotY, float rotZ, float sc, bool corner = false) {
			setModel(prog, trans, rotX, rotY, rotZ, vec3(sc), corner);
		}
		void setModel(shared_ptr<Program> prog, vec3 trans, float rotX, float rotY, float rotZ, vec3 sc, bool corner = false) {

			// Calculate initial transformations
			vec3 initial_translate;
			float initial_scale;
			if (corner) {
				initial_translate = -min;
				initial_scale = 2.0f/length(max - min);
			} else {
				initial_translate = -(min + (0.5f * (max - min)));
				initial_scale = 2.0f/length(max - min);
			}

			
			mat4 Init_Trans = glm::translate( glm::mat4(1.0f), initial_translate);
			mat4 Init_Scale = glm::scale(glm::mat4(1.0f), vec3(initial_scale));

			// Added transformations
			mat4 Trans = glm::translate( glm::mat4(1.0f), trans);
			mat4 RotX = glm::rotate( glm::mat4(1.0f), rotX * TO_RAD, vec3(1, 0, 0));
			mat4 RotY = glm::rotate( glm::mat4(1.0f), rotY * TO_RAD, vec3(0, 1, 0));
			mat4 RotZ = glm::rotate ( glm::mat4(1.0f), rotZ * TO_RAD, vec3(0, 0, 1));
			mat4 ScaleS = glm::scale(glm::mat4(1.0f), sc);

			// Combine all matricies
			mat4 initial = Init_Scale*Init_Trans;
			mat4 rotation = RotX*RotY*RotZ;
			mat4 ctm = Trans*rotation*ScaleS;
			mat4 combined = ctm*initial;
			transform = combined;

			// Set model matrix for prog
			glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, value_ptr(combined));
		}
		void draw(shared_ptr<Program>& prog) {
			for (int i = 0; i < shapes.size(); i++) {
				shapes[i]->draw(prog);
			}
		}
		void drawAll(shared_ptr<Program>& prog) {
			for (int i = 0; i < instances.size(); i++) {
				setModel(prog, instances[i]);
				draw(prog);
			}
		}
		
		void drawInstances(shared_ptr<Program>& prog, GLint pointsBuffer, unsigned int numInstances) {
			shapes[0]->drawInstances(prog, pointsBuffer, numInstances);
		}
		
		// get Bounding Boxes for the last instance drawn
		vector<BoundingBox> getBoundingBoxes() {
			vector<BoundingBox> boxes;
			for (int i = 0; i < shapes.size(); i++) {
				shared_ptr<Shape> shape = shapes[i];
				vec4 transformed_min = transform * vec4(shape->min, 1.0f);
				vec4 transformed_max = transform * vec4(shape->max, 1.0f);
				boxes.push_back(BoundingBox(transformed_min, transformed_max));
			}
			return boxes;
		}

		// get Bounding Boxes for the ith instance
		vector<BoundingBox> getBoundingBoxes(shared_ptr<Program>& prog, int i) {
			vector<BoundingBox> boxes;
			setModel(prog, instances[i]);
			return getBoundingBoxes();
		}
};


class Application : public EventCallbacks
{

public:

	WindowManager * windowManager = nullptr;

	// Our shader program - use this one for Blinn-Phong
	std::shared_ptr<Program> prog;

	//Our shader program for textures
	std::shared_ptr<Program> texProg;

	shared_ptr<Shape> dog;

	//global data for ground plane - direct load constant defined CPU data to GPU (not obj)
	GLuint GrndBuffObj, GrndNorBuffObj, GrndTexBuffObj, GIndxBuffObj;
	GLuint pointsBuf;

	GLuint boxParticlesID;
	GLuint netParticlesID;
	GLfloat boxParticles;
	GLfloat netParticles;

	Body cubeBody, ropeBody;

	int g_GiboLen;
	//ground VAO
	GLuint GroundVertexArrayID;

	//the image to use as a texture (ground)
	shared_ptr<Texture> texture0;
	shared_ptr<Texture> texture1;	
	shared_ptr<Texture> texture2;
	shared_ptr<Texture> texture3, underwater_tex;

	//vector<Particle> cubeParticles;
	//vector<Particle> ropeParticles;

	Spline splinepath[2];
	vec3 g_eye = vec3(0, 1, 0);
	vec3 g_lookAt = vec3(0, 0, -1);
	bool goCamera = false;

	

	int mat = 0;

	CompoundShape cube, rope, sphere, bunnyNoNorm, icoNoNorm;

	//global data (larger program should be encapsulated)
	vec3 gMin;
	float gRot = 0;
	float gCamH = 0;
	//animation data
	float lightTrans = 0;
	float gTrans = -3;
	float sTheta = 0;
	float eTheta = 0;
	float hTheta = 0;
	float lightX = 0;
	float pitch = 0;
	float yaw = 0;
	bool g_pressed = false;

	void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
	{
		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		{
			glfwSetWindowShouldClose(window, GL_TRUE);
		}
		
		if (key == GLFW_KEY_W && action == GLFW_PRESS)
		{
			mycam.w = 1;
		}
		if (key == GLFW_KEY_W && action == GLFW_RELEASE)
		{
			mycam.w = 0;
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS)
		{
			mycam.s = 1;
		}
		if (key == GLFW_KEY_S && action == GLFW_RELEASE)
		{
			mycam.s = 0;
		}
		if (key == GLFW_KEY_A && action == GLFW_PRESS)
		{
			mycam.a = 1;
		}
		if (key == GLFW_KEY_A && action == GLFW_RELEASE)
		{
			mycam.a = 0;
		}
		if (key == GLFW_KEY_D && action == GLFW_PRESS)
		{
			mycam.d = 1;
		}
		if (key == GLFW_KEY_D && action == GLFW_RELEASE)
		{
			mycam.d = 0;
		}
		if (key == GLFW_KEY_M && action == GLFW_PRESS)
		{
			mat = (mat + 1) % 3;
		}
		if (key == GLFW_KEY_E && action == GLFW_PRESS)
		{
			lightX += 0.5;
		}
		if (key == GLFW_KEY_Q && action == GLFW_PRESS)
		{
			lightX -= 0.5;
		}
		if (key == GLFW_KEY_G && action == GLFW_PRESS)
		{
			g_pressed = true;
			//goCamera = true;
			//initSplinePath();
		}
	}

	void mouseCallback(GLFWwindow *window, int button, int action, int mods)
	{
		double posX, posY;

		if (action == GLFW_PRESS)
		{
			 glfwGetCursorPos(window, &posX, &posY);
			 cout << "Pos X " << posX <<  " Pos Y " << posY << endl;
		}
		
	}

	void scrollCallback(GLFWwindow *window, double deltaX, double deltaY) {
		
		int pitch_min = -80;
		int pitch_max = 80;

		int width, height;
		glfwGetFramebufferSize(windowManager->getHandle(), &width, &height);
		
		int sensitivity = 10;

		mycam.pitch += deltaY * sensitivity;
		mycam.yaw += deltaX * sensitivity;

		if (mycam.pitch < pitch_min) {
			mycam.pitch = pitch_min;
		} else if (mycam.pitch > pitch_max) {
			mycam.pitch = pitch_max;
		}


	}

	void resizeCallback(GLFWwindow *window, int width, int height)
	{
		glViewport(0, 0, width, height);
	}

	// Code to load in the three textures 
	void initTex(const std::string& resourceDirectory){  
		texture0 = make_shared<Texture>();  
		texture0->setFilename(resourceDirectory + "/crate.jpg"); 
		texture0->init(); 
		texture0->setUnit(0); 
		texture0->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); 

		texture1 = make_shared<Texture>(); 
		texture1->setFilename(resourceDirectory + "/world.jpg"); 
		texture1->init();  
		texture1->setUnit(1); 
		texture1->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); 
		
		texture2 = make_shared<Texture>(); 
		texture2->setFilename(resourceDirectory + "/cartoonSky.png"); 
		texture2->init(); 
		texture2->setUnit(2); 
		texture2->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); 

		texture3 = make_shared<Texture>(); 
		texture3->setFilename(resourceDirectory + "/flowers.jpg"); 
		texture3->init(); 
		texture3->setUnit(2); 
		texture3->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); 

		underwater_tex = make_shared<Texture>(); 
		underwater_tex->setFilename(resourceDirectory + "/underwater.jpeg"); 
		underwater_tex->init(); 
		underwater_tex->setUnit(2); 
		underwater_tex->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); 
	} 


	void init(const std::string& resourceDirectory)
	{
		GLSL::checkVersion();

		// Set background color.
		glClearColor(1,1,1,1);
		// Enable z-buffer test.
		glEnable(GL_DEPTH_TEST);

		// Initialize the GLSL program that we will use for local shading
		prog = make_shared<Program>();
		prog->setVerbose(true);
		prog->setShaderNames(resourceDirectory + "/simple_vert.glsl", resourceDirectory + "/simple_frag.glsl");
		prog->init();
		prog->addUniform("P");
		prog->addUniform("V");
		prog->addUniform("M");
		prog->addUniform("flip");
		prog->addUniform("MatAmb");
		prog->addUniform("MatDif");
		prog->addUniform("MatSpec");
		prog->addUniform("MatShine");
		prog->addUniform("lightPos");
		prog->addAttribute("vertPos");
		prog->addAttribute("vertNor");
		prog->addAttribute("vertTex");
		prog->addAttribute("instancePos");

		// Initialize the GLSL program that we will use for texture mapping
		texProg = make_shared<Program>();
		texProg->setVerbose(true);
		texProg->setShaderNames(resourceDirectory + "/tex_vert.glsl", resourceDirectory + "/tex_frag0.glsl");
		texProg->init();
		texProg->addUniform("P");
		texProg->addUniform("V");
		texProg->addUniform("M");
		texProg->addUniform("flip");
		texProg->addUniform("Texture0");
		texProg->addUniform("MatShine");
		texProg->addUniform("lightPos");
		texProg->addUniform("shade");
		texProg->addAttribute("vertPos");
		texProg->addAttribute("vertNor");
		texProg->addAttribute("vertTex");

		initTex(resourceDirectory);

		initSplinePath();

	}

	void initSplinePath() {
		// init splines up and down
		splinepath[0] = Spline(glm::vec3(1,0,1), glm::vec3(1,0,0.5), glm::vec3(1, 0, -0.5), glm::vec3(1, 0, -1.5), 5);
		splinepath[1] = Spline(glm::vec3(1, 0, -1.5), glm::vec3(1,0.05,-1.8), glm::vec3(1, 0.1, -2.2), glm::vec3(1,0.25,-2.3), 5);
	}

	void initGeom(const std::string& resourceDirectory)
	{
		//EXAMPLE set up to read one shape from one obj file - convert to read several
		// Initialize mesh
		// Load geometry
 		// Some obj files contain material information.We'll ignore them for this assignment.
 		
		//load in the mesh and make the shape(s)
 		
		//read out information stored in the shape about its size - something like this...
		//then do something with that information.....



		sphere.load(resourceDirectory + "/sphereWTex.obj");
		rope.load(resourceDirectory + "/ropeOBJ.obj");
		bunnyNoNorm.load(resourceDirectory + "/bunnyNoNorm.obj");
		icoNoNorm.load(resourceDirectory + "/icoNoNormals.obj");

		generateRopeBody(ropeBody, vec3(0,0,-2), 7, 8);
		generateCubeBody(cubeBody, vec3(-0.15,0,-1), vec3(0, 0, -0.2), 8);

	}

     //helper function to pass material data to the GPU
	void SetMaterial(shared_ptr<Program> curS, int i) {

    	switch (i) {
    		case 0: //shiny blue plastic
    			glUniform3f(curS->getUniform("MatAmb"), 0.096, 0.046, 0.095);
    			glUniform3f(curS->getUniform("MatDif"), 0.0, 0.46, 0.95);
    			glUniform3f(curS->getUniform("MatSpec"), 0.45, 0.23, 0.45);
    			glUniform1f(curS->getUniform("MatShine"), 120.0);
    		break;
    		case 1: // flat grey
    			glUniform3f(curS->getUniform("MatAmb"), 0.063, 0.038, 0.1);
    			glUniform3f(curS->getUniform("MatDif"), 0.33, 0.9, 1.0);
    			glUniform3f(curS->getUniform("MatSpec"), 0.3, 0.2, 0.5);
    			glUniform1f(curS->getUniform("MatShine"), 0);
    		break;
    		case 2: //brass
    			glUniform3f(curS->getUniform("MatAmb"), 0.004, 0.05, 0.09);
    			glUniform3f(curS->getUniform("MatDif"), 0.9, 0.5, 0.1);
    			glUniform3f(curS->getUniform("MatSpec"), 0.02, 0.25, 0.45);
    			glUniform1f(curS->getUniform("MatShine"), 27.9);
    		break;
  		}
	}

	/* helper function to set model trasnforms */
  	void SetModel(vec3 trans, float rotY, float rotX, float sc, shared_ptr<Program> curS) {
  		mat4 Trans = glm::translate( glm::mat4(1.0f), trans);
  		mat4 RotX = glm::rotate( glm::mat4(1.0f), rotX, vec3(1, 0, 0));
  		mat4 RotY = glm::rotate( glm::mat4(1.0f), rotY, vec3(0, 1, 0));
  		mat4 ScaleS = glm::scale(glm::mat4(1.0f), vec3(sc));
  		mat4 ctm = Trans*RotX*RotY*ScaleS;
  		glUniformMatrix4fv(curS->getUniform("M"), 1, GL_FALSE, value_ptr(ctm));
  	}

	void setModel(std::shared_ptr<Program> prog, std::shared_ptr<MatrixStack>M) {
		glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, value_ptr(M->topMatrix()));
   	}
	
	/* camera controls - do not change */
	void SetView(shared_ptr<Program>  shader) {
  		glm::mat4 Cam = lookAt(g_eye, g_lookAt, vec3(0, 1, 0));
  		glUniformMatrix4fv(shader->getUniform("V"), 1, GL_FALSE, value_ptr(Cam));
	}

	void updateUsingCameraPath(float frametime)  {

		if(!splinepath[0].isDone()){
			splinepath[0].update(frametime * 0.5);
			g_eye = splinepath[0].getPosition();
		} else {
			splinepath[1].update(frametime * 0.2);
			g_eye = splinepath[1].getPosition();
			if (splinepath[1].isDone()) {
				goCamera = false;
				initSplinePath();
			}
		}
   	}

	vec3 getCenterOfMass(vector<vec3> particles) {
		vec3 aggregate = vec3(0);
		for (int i = 0; i < particles.size(); i++) {
			aggregate += particles[i];
		}
		return (1.0f / particles.size()) * aggregate;
	}

	void generateRopeBody(Body &body, vec3 pos, int grid_spacing, int grid_size) {

		// const float z_pos = pos.z;
		// const float x_min = pos.x + -0.5 * (grid_size * grid_spacing + 1) * PARTICLE_SIZE;
		// const float y_min = pos.y + -0.5 * (grid_size * grid_spacing + 1) * PARTICLE_SIZE;

		vector<vec3> particles;
		// for (int x = 0; x <= grid_size * grid_spacing; x++) {
		// 	for (int y = 0; y <= grid_size * grid_spacing; y++) {
		// 		float x_pos = x_min + (x + 0.5) * PARTICLE_SIZE;
		// 		float y_pos = y_min + (y + 0.5) * PARTICLE_SIZE;
		// 		if (x % grid_spacing == 0 || y % grid_spacing == 0) {
		// 			particles.push_back(vec3(x_pos, y_pos, z_pos));
		// 		}
		// 	}
		// }
		particles.push_back(pos);

		body.mass = particles.size();
		body.velocity = vec3(0);
		body.particles = getParticleBuf(particles);
		body.type = STATIONARY;
		body.center_of_mass = getCenterOfMass(particles);
		body.rotational_velocity = vec3(0);
		body.rotation = vec3(0);
		glGenBuffers(1, &body.particlesID);
		glBindBuffer(GL_ARRAY_BUFFER, body.particlesID);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * particles.size() * 3, NULL, GL_STREAM_DRAW);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * particles.size() * 3, body.particles);
	}

	void generateCubeBody(Body &body, vec3 pos, vec3 velocity, int size) {

		float x_min = pos.x - (0.5 * (size - 0.5) * PARTICLE_SIZE);
		float y_min = pos.y - (0.5 * (size - 0.5) * PARTICLE_SIZE);
		float z_min = pos.z - (0.5 * (size - 0.5) * PARTICLE_SIZE);

		vector<vec3> particles;

		for (int x = 0; x < size; x++) {
			for (int y = 0; y < size; y++) {
				for (int z = 0; z < size; z++) {
					float x_pos = x_min + x * PARTICLE_SIZE;
					float y_pos = y_min + y * PARTICLE_SIZE;
					float z_pos = z_min + z * PARTICLE_SIZE;
					particles.push_back(vec3(x_pos, y_pos, z_pos));
				}
			}
		}

		body.mass = particles.size();
		body.velocity = velocity;
		body.particles = getParticleBuf(particles);
		body.type = MOVING;
		body.center_of_mass = getCenterOfMass(particles);
		body.rotational_velocity = vec3(0);
		body.rotation = vec3(0);
		glGenBuffers(1, &body.particlesID);
		glBindBuffer(GL_ARRAY_BUFFER, body.particlesID);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * particles.size() * 3, NULL, GL_STREAM_DRAW);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * particles.size() * 3, body.particles);
	}

	void render() {

		if (g_pressed) {
			g_pressed = false;
			generateCubeBody(cubeBody, vec3(0,0,-1), vec3(0, 0, -0.2), 8);
		}

		double frametime = get_last_elapsed_time();
		float current = 0.5;

		if (goCamera) {
			updateUsingCameraPath(frametime);
		}

		// Get current frame buffer size.
		int width, height;
		glfwGetFramebufferSize(windowManager->getHandle(), &width, &height);
		glViewport(0, 0, width, height);

		// Clear framebuffer.
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Use the matrix stack for Lab 6
		float aspect = width/(float)height;

		// Create the matrix stacks - please leave these alone for now
		auto Projection = make_shared<MatrixStack>();
		mat4 View = mycam.process(frametime);



		
		
		
			
		auto Model = make_shared<MatrixStack>();

		// Apply perspective projection.
		Projection->pushMatrix();
		Projection->perspective(45.0f, aspect, 0.01f, 100.0f);

		//prog->bind();
		texProg->bind();
		glUniformMatrix4fv(texProg->getUniform("P"), 1, GL_FALSE, value_ptr(Projection->topMatrix()));
		if (!goCamera) {
			glUniformMatrix4fv(texProg->getUniform("V"), 1, GL_FALSE, value_ptr(View));
		} else {
			SetView(texProg);
		}
		glUniform3f(texProg->getUniform("lightPos"), 1.5+lightX, 2.0, 2.9);
		glUniform1i(texProg->getUniform("flip"), 1); // flip normals if needed
		glUniform1i(texProg->getUniform("shade"), 1);

		underwater_tex->bind(texProg->getUniform("Texture0"));

		glDisable(GL_DEPTH_TEST);
		if (!goCamera) {
			sphere.setModel(texProg, -mycam.pos, 0,180,0, vec3(0.05));
		} else {
			sphere.setModel(texProg, g_eye, 0,180,0, vec3(0.05));
		}
		sphere.draw(texProg);
		glEnable(GL_DEPTH_TEST);

		glUniform1i(texProg->getUniform("flip"), 0); // flip normals if needed
		glUniform1i(texProg->getUniform("shade"), 1);
		
		texture0->bind(texProg->getUniform("Texture0"));
		//rope.addInstance(vec3(0.01,0,-1), vec3(-90, 90, 0), vec3(1));
		//rope.drawAll(texProg);


		//texture2->bind(texProg->getUniform("Texture0"));

		prog->bind();
		glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, value_ptr(Projection->topMatrix()));
		glUniform3f(prog->getUniform("lightPos"), 1.5+lightX, 2.0, 2.9);
		glUniform1i(prog->getUniform("flip"), 0); // flip normals if needed
		if (!goCamera) {
			glUniformMatrix4fv(prog->getUniform("V"), 1, GL_FALSE, value_ptr(View));
		} else {
			updateUsingCameraPath(frametime);
			SetView(prog);
		}

		SetMaterial(prog, mat);



		
		// for (int i = 0; i < cubeParticles.size() + ropeParticles.size(); i++) {
		// 	for (int j = i+1; j < cubeParticles.size() + ropeParticles.size(); j++) {
		// 		particleRef(i)->checkCollision(*particleRef(j));
		// 	}
		// 	if (i < cubeParticles.size()) {
		// 		cubeParticles[i].updatePos(frametime);
		// 	}
		// }

		cubeBody.checkCollision(ropeBody);
		//cubeBody.velocity += (float)frametime * vec3(0,0,-0.05);
		sphere.setModel(prog, cubeBody.center_of_mass, 0,0,0, PARTICLE_SIZE);
		sphere.draw(prog);
		cubeBody.updatePos(frametime);
		drawParticles(cubeBody);
		drawParticles(ropeBody);

		prog->unbind();

	}

	GLfloat *getParticleBuf(vector<vec3> particle_vector) {
		GLfloat *particleBuf = (GLfloat *) malloc(sizeof(GLfloat) * particle_vector.size() * 3);
		if (particleBuf == NULL) {
			cout << "Malloc Error" << endl;
			exit(1);
		}

		vec3 pos;
		for (int i = 0; i < particle_vector.size(); i++) {
			pos = particle_vector[i];
			particleBuf[i*3+0] = pos.x; 
			particleBuf[i*3+1] = pos.y; 
			particleBuf[i*3+2] = pos.z; 
				
		} 
		return particleBuf;
	}

	void drawParticles(Body &body) {

		unsigned int buf_size = body.mass * 3 * sizeof(GLfloat);
		glBindBuffer(GL_ARRAY_BUFFER, body.particlesID);
		glBufferData(GL_ARRAY_BUFFER, buf_size, NULL, GL_STREAM_DRAW);
		glBufferSubData(GL_ARRAY_BUFFER, 0, buf_size, body.particles);
		
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		sphere.setModel(prog, vec3(0), 0, 0, 0, PARTICLE_SIZE / 1.13);
		sphere.drawInstances(prog, body.particlesID, body.mass);

	}

};

int main(int argc, char *argv[])
{
	// Where the resources are loaded from
	std::string resourceDir = "../resources";

	if (argc >= 2)
	{
		resourceDir = argv[1];
	}

	Application *application = new Application();

	// Your main will always include a similar set up to establish your window
	// and GL context, etc.

	WindowManager *windowManager = new WindowManager();
	windowManager->init(640, 480);
	windowManager->setEventCallbacks(application);
	application->windowManager = windowManager;

	// This is the code that will likely change program to program as you
	// may need to initialize or set up different data and state

	application->init(resourceDir);
	application->initGeom(resourceDir);

	// Loop until the user closes the window.
	while (! glfwWindowShouldClose(windowManager->getHandle()))
	{
		// Render scene.
		application->render();

		// Swap front and back buffers.
		glfwSwapBuffers(windowManager->getHandle());
		// Poll for and process events.
		glfwPollEvents();
	}

	// Quit program.
	windowManager->shutdown();
	return 0;
}
