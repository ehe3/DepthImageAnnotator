#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
#include "glm/gtc/quaternion.hpp"
#include <glm/gtx/euler_angles.hpp>

#include <iostream>
#include <limits>
#include <random>
#include <cstdlib>

#include "SkeletonModel.h"
#include "Reinitializer.h"

#define INT2VOIDP(i) (void*)(uintptr_t)(i)

struct Box 
{
	public:
		int x;
		int y;
		int width;
	
		Box(int x, int y, int width) : x{x}, y{y}, width{width} {}
};

struct Intrinsics
{
	public:
		float ppx;
		float ppy; 
		float fx;
		float fy;
		float left;
		float right;
		float bottom;
		float top;
		float zNear;
		float zFar;

		Intrinsics(float ppx, float ppy, float fx, float fy, float left, float right, float bottom, float top, float zNear, float zFar) : ppx{ppx}, ppy{ppy}, fx{fx}, fy{fy}, left{left}, right{right}, bottom{bottom}, top{top}, zNear{zNear}, zFar{zFar} {}
};

struct PoseParameters
{
	public:
		float XTranslation;
		float YTranslation;
		float ZTranslation;
		glm::quat GlobalQuat;
		float ToeXRot;
		float LegXRot;
		float LegZRot;
		float Scale;

		PoseParameters() : XTranslation{0.0f}, YTranslation{0.0f}, ZTranslation{0.0f}, GlobalQuat{glm::quat(1.0, 0.0, 0.0, 0.0)}, ToeXRot{0.0f}, LegXRot{0.0f}, LegZRot{0.0f}, Scale{0.0f} {}

		PoseParameters(float xtrans, float ytrans, float ztrans, glm::quat GQuat, float toexrot, float legxrot, float legzrot, float scale) : XTranslation{xtrans}, YTranslation{ytrans}, ZTranslation{ztrans}, GlobalQuat{GQuat}, ToeXRot{toexrot}, LegXRot{legxrot}, LegZRot{legzrot}, Scale{scale} {}

		PoseParameters(float xtrans, float ytrans, float ztrans, float quatw, float quatx, float quaty, float quatz, float toexrot, float legxrot, float legzrot, float scale) : XTranslation{xtrans}, YTranslation{ytrans}, ZTranslation{ztrans}, GlobalQuat{glm::quat(quatw, quatx, quaty, quatz)}, ToeXRot{toexrot}, LegXRot{legxrot}, LegZRot{legzrot}, Scale{scale} {}

		PoseParameters(const PoseParameters &params) : XTranslation{params.XTranslation}, YTranslation{params.YTranslation}, ZTranslation{params.ZTranslation}, GlobalQuat{params.GlobalQuat}, ToeXRot{params.ToeXRot}, LegXRot{params.LegXRot}, LegZRot{params.LegZRot}, Scale{params.Scale} {}

		float GetQuatW() const
		{
			return GlobalQuat.w;
		}

		float GetQuatX() const
		{
			return GlobalQuat.x;
		}

		float GetQuatY() const
		{
			return GlobalQuat.y; 
		}

		float GetQuatZ() const
		{
			return GlobalQuat.z;
		}

		PoseParameters operator+(PoseParameters const &obj) const
		{
			glm::quat rot_combined = obj.GlobalQuat * GlobalQuat;
			return PoseParameters(XTranslation + obj.XTranslation, YTranslation + obj.YTranslation, ZTranslation + obj.ZTranslation, glm::normalize(rot_combined), ToeXRot + obj.ToeXRot, LegXRot + obj.LegXRot, LegZRot + obj.LegZRot, Scale + obj.Scale);	
		}

		PoseParameters operator-(PoseParameters const &obj) const
		{
			glm::quat rot_diff = GlobalQuat*glm::inverse(obj.GlobalQuat);
			return PoseParameters(XTranslation - obj.XTranslation, YTranslation - obj.YTranslation, ZTranslation - obj.ZTranslation, glm::normalize(rot_diff), ToeXRot - obj.ToeXRot, LegXRot - obj.LegXRot, LegZRot - obj.LegZRot, Scale - obj.Scale);	
		}

		PoseParameters operator*(float c) const
		{
			glm::quat rot_scaled = glm::mix(glm::quat(1.0, 0.0, 0.0, 0.0), GlobalQuat, c); 
			return PoseParameters(c * XTranslation, c * YTranslation, c * ZTranslation, glm::normalize(rot_scaled), c * ToeXRot, c * LegXRot, c * LegZRot, c * Scale);
		}

		void AssuageVelocity()
		{
			float translationIncrement =  0.02;
			float angleIncrement = glm::radians(5.0);
			float localAngleIncrement = glm::radians(3.0);
			float scaleIncrement = 0.005;
			XTranslation = XTranslation < -translationIncrement ? -translationIncrement : XTranslation > translationIncrement ? translationIncrement : XTranslation;	
			YTranslation = YTranslation < -translationIncrement ? -translationIncrement : YTranslation > translationIncrement ? translationIncrement : YTranslation;	
			ZTranslation = ZTranslation < -translationIncrement ? -translationIncrement : ZTranslation > translationIncrement ? translationIncrement : ZTranslation;	
			ToeXRot = ToeXRot < -localAngleIncrement ? -localAngleIncrement : ToeXRot > localAngleIncrement ? localAngleIncrement : ToeXRot;
			LegXRot = LegXRot < -localAngleIncrement ? -localAngleIncrement : LegXRot > localAngleIncrement ? localAngleIncrement : LegXRot;
			LegZRot = LegZRot < -localAngleIncrement ? -localAngleIncrement : LegZRot > localAngleIncrement ? localAngleIncrement : LegZRot;
			glm::vec3 eulers = glm::eulerAngles(GlobalQuat);
			eulers[0] = eulers[0] < -angleIncrement ? -angleIncrement : eulers[0] > angleIncrement ? angleIncrement : eulers[0];
			eulers[1] = eulers[1] < -angleIncrement ? -angleIncrement : eulers[1] > angleIncrement ? angleIncrement : eulers[1];
			eulers[2] = eulers[2] < -angleIncrement ? -angleIncrement : eulers[2] > angleIncrement ? angleIncrement : eulers[2];
			GlobalQuat = glm::quat(eulers);
			Scale = Scale < -scaleIncrement ? -scaleIncrement : Scale > scaleIncrement ? scaleIncrement : Scale;
		}

		void AssuagePosition(float toeXMin=glm::radians(-15.0f), float toeXMax=glm::radians(15.0f), float legXMin=glm::radians(-20.0f), float legXMax=glm::radians(30.0f), float legZMin=glm::radians(-30.0f), float legZMax=glm::radians(30.0f))
		{
			ToeXRot = ToeXRot < toeXMin ? toeXMin : ToeXRot > toeXMax ? toeXMax : ToeXRot;
			LegXRot = LegXRot < legXMin ? legXMin : LegXRot > legXMax ? legXMax : LegXRot;
			LegZRot = LegZRot < legZMin ? legZMin : LegZRot > legZMax ? legZMax : LegZRot;
		}

		// For debugging only
		void Print() const
		{
			std::cout << "XTranslation: " << XTranslation << " YTranslation: " << YTranslation << " ZTranslation: " << ZTranslation << " GlobalQuatW: " << GlobalQuat.w << " GlobalQuatX: " << GlobalQuat.x << " GlobalQuatY: " << GlobalQuat.y << " GlobalQuatZ: " << GlobalQuat.z <<  " ToeXRot: " << ToeXRot << " LegXRot: " << LegXRot << " LegZRot: " << LegZRot << " Scale: " << Scale << std::endl;
		}

		// For debugging only
		std::string CommaSeperatedString() const
		{
			std::string output = std::to_string(XTranslation) + "," + std::to_string(YTranslation) + "," + std::to_string(ZTranslation) + "," + std::to_string(GlobalQuat.w) + "," + std::to_string(GlobalQuat.x) + "," + std::to_string(GlobalQuat.y) + "," + std::to_string(GlobalQuat.z) + "," + std::to_string(ToeXRot) + "," + std::to_string(LegXRot) + "," + std::to_string(LegZRot) + "," + std::to_string(Scale) + ",";
			return output;
		}
};

struct EnergyPosePair
{
	PoseParameters position;
	float energy;

	EnergyPosePair(PoseParameters position, float energy) : position{position}, energy{energy} {}
};

// a candidate of the PSO algorithm (single pose)
class Particle {

	public:
		PoseParameters Position;
		float BestEnergyScore;
		PoseParameters BestPosition;
		PoseParameters Velocity;

		Particle(): Position{PoseParameters()}, BestEnergyScore{std::numeric_limits<float>::infinity()}, BestPosition{PoseParameters()}, Velocity{PoseParameters()} {}
		Particle(PoseParameters position): Position{position}, BestEnergyScore{std::numeric_limits<float>::infinity()}, BestPosition{position}, Velocity{PoseParameters()} {}
};

class PSO {

	private:	
		int NumParticles;
		std::vector<Particle> Particles;
		float CognitiveConst, SocialConst, ConstrictionConst; // PSO population constants
		// OpenGL vars
		GLFWwindow* window;
		glm::mat4 MeshToBoneToe_L, MeshToBoneLeg_L, BoneToMeshToe_L, BoneToMeshLeg_L; // bone transformation matricies left foot
		glm::mat4 MeshToBoneToe_R, MeshToBoneLeg_R, BoneToMeshToe_R, BoneToMeshLeg_R; // bone transformation matricies right foot
		Shader RepeatShader, SubtractionShader, RTTShader, R2Shader, PTShader, ModelShader;
		SkeletonModel footSkeleton_L;
		SkeletonModel footSkeleton_R;
		// quads, textures, and buffers
		GLuint quadVAO, quadVBO, repeatQuadLargeVAO, repeatQuadLargeVBO, refdepthtex, peng, repeattex, ping, depthtexture, pong, difftex, pling, tex16, plang, tex8, plong, tex4, plung, tex2, pleng, tex1;
		// picture
		GLuint picholder, pictex;
		// instance buffers left foot
		GLuint instanceVBO_L, transformationInstanceBuffer_L, rottoeVB_L, rotlegVB_L;
		// instance buffers right foot
		GLuint instanceVBO_R, transformationInstanceBuffer_R, rottoeVB_R, rotlegVB_R;
		// sampling variables
		glm::quat quatx, quaty, quatz, quati; // standard quaternions representing 90 degree rotations
		float quatx_stdv, quaty_stdv, quatz_stdv; // standard deviations for uniform quaternion sampling
		float quatx_stdv_long, quaty_stdv_long, quatz_stdv_long; // standard deviations for uniform quaternion sampling long
		std::uniform_real_distribution<float> quat_x, quat_y, quat_z; // uniform quaternion distributions
		std::uniform_real_distribution<float> quat_x_long, quat_y_long, quat_z_long;
		float st, lsr, ss; // standard deviations for translation and scale sampling
		std::uniform_real_distribution<float> t; // uniform translation distribution
		std::uniform_real_distribution<float> t_dim;
		std::uniform_real_distribution<float> t_z;
		std::uniform_real_distribution<float> lr; // uniform local rotation distribution
		std::uniform_real_distribution<float> sc; // uniform scale distribution
		std::uniform_real_distribution<float> toexrot; // uniform toe x rotation sampling
		std::uniform_real_distribution<float> legxrot; // uniform leg x rotation sampling
		std::uniform_real_distribution<float> legzrot; // uniform leg z rotation sampling
		std::mt19937 gen; // prng
		
	private:
		// sets up vertex attributes (model, toe, leg) for a skeleton model
		static void ConfigureSkeleton(SkeletonModel& model, float* translations, int numInstances, GLuint& instanceVBO, GLuint& transformationInstanceBuffer, GLuint& rottoeVB, GLuint& rotlegVB) 
		{
			glBindVertexArray(model.meshes[0].VAO);

			// generate buffer to hold tile offsets (separate case)
			glGenBuffers(1, &instanceVBO);
			glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numInstances, &translations[0], GL_STATIC_DRAW);

			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), INT2VOIDP(0));
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(2, 1);
		
			// set up the instance VBO for model matricies 
			glGenBuffers(1, &transformationInstanceBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, transformationInstanceBuffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4)*numInstances, nullptr, GL_STATIC_DRAW);

			glEnableVertexAttribArray(3);
			glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(0));
			glEnableVertexAttribArray(4);
			glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(sizeof(glm::vec4)));
			glEnableVertexAttribArray(5);
			glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(2*sizeof(glm::vec4)));
			glEnableVertexAttribArray(6);
			glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(3*sizeof(glm::vec4)));
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(3, 1);
			glVertexAttribDivisor(4, 1);
			glVertexAttribDivisor(5, 1);
			glVertexAttribDivisor(6, 1);

			// set up the instance VBO for toe matricies
			glGenBuffers(1, &rottoeVB);
			glBindBuffer(GL_ARRAY_BUFFER, rottoeVB);
			glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4)*numInstances, nullptr, GL_STATIC_DRAW);

			glEnableVertexAttribArray(7);
			glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(0));
			glEnableVertexAttribArray(8);
			glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(sizeof(glm::vec4)));
			glEnableVertexAttribArray(9);
			glVertexAttribPointer(9, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(2*sizeof(glm::vec4)));
			glEnableVertexAttribArray(10);
			glVertexAttribPointer(10, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(3*sizeof(glm::vec4)));
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(7, 1);
			glVertexAttribDivisor(8, 1);
			glVertexAttribDivisor(9, 1);
			glVertexAttribDivisor(10, 1);

			// set up the instance VBO for leg matricies left foot
			glGenBuffers(1, &rotlegVB);
			glBindBuffer(GL_ARRAY_BUFFER, rotlegVB);
			glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4)*numInstances, nullptr, GL_STATIC_DRAW);

			glEnableVertexAttribArray(11);
			glVertexAttribPointer(11, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(0));
			glEnableVertexAttribArray(12);
			glVertexAttribPointer(12, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(sizeof(glm::vec4)));
			glEnableVertexAttribArray(13);
			glVertexAttribPointer(13, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(2*sizeof(glm::vec4)));
			glEnableVertexAttribArray(14);
			glVertexAttribPointer(14, 4, GL_FLOAT, GL_FALSE, 4*sizeof(glm::vec4), INT2VOIDP(3*sizeof(glm::vec4)));
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glVertexAttribDivisor(11, 1);
			glVertexAttribDivisor(12, 1);
			glVertexAttribDivisor(13, 1);
			glVertexAttribDivisor(14, 1);
		}

		// generates a stand alone texture with no data, to be filled in later, used for input depth images
		static void GenerateTexture(GLuint &texID, int texNum, int texWidth, int texHeight)
		{
			glGenTextures(1, &texID);
			glActiveTexture(GL_TEXTURE0 + texNum);
			glBindTexture(GL_TEXTURE_2D, texID);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, texWidth, texHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);	
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		}

		// generates a texture that is attached to a framebuffer, used for rendering to textures
		static void GenerateTextureWithFramebuffer(GLuint &fboID, GLuint &texID, int texNum, int texWidth, int texHeight)
		{
			// first set up the framebuffer
			glGenFramebuffers(1, &fboID);
			glBindFramebuffer(GL_FRAMEBUFFER, fboID);

			// next set up the texture
			glGenTextures(1, &texID);
			glActiveTexture(GL_TEXTURE0 + texNum);
			glBindTexture(GL_TEXTURE_2D, texID);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, texWidth, texHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texID, 0);
		}

	public:
		PSO(float CogConst=1.3, float SocConst=2.8) : 
			NumParticles{32}, // note this must divide into 1280 evenly	
			CognitiveConst{CogConst}, 
			SocialConst{SocConst}, 
			ConstrictionConst{0.0f}, 
			window{nullptr}
		{
			// Set the initial size of the Particle vector
			Particles.resize(NumParticles);

			// Make sure constructed constriction constant is valid
			float Phi = CognitiveConst + SocialConst;
			if (Phi <= 4) std::cerr << "WARNING: Optimization constants too small" << std::endl;
			ConstrictionConst = 2.0f / std::abs(2.0f - Phi - sqrt(Phi*Phi-4*Phi));

			// Initialize GLFW
			if (!glfwInit()) std::cerr << "WARNING: GLFW not initialized properly" << std::endl;
			window = glfwCreateWindow(32*NumParticles, 32, "PSO", NULL, NULL);

			// Check to see if the window is valid
			if (!window)
			{
				glfwTerminate();
				std::cerr << "WARNING: GLFW window was not created properly" << std::endl;
			}

			// Make the window's context current and then hide it
			glfwMakeContextCurrent(window);
			glfwHideWindow(window);

			// Initialize GLEW
			glewExperimental = true;
			if (glewInit() != GLEW_OK) std::cerr << "WARNING: GLEW not initialized properly" << std::endl;

			// Set up sampling variables
			quatx = glm::normalize(glm::quat(1.0, 1.0, 0.0, 0.0));
			quaty = glm::normalize(glm::quat(1.0, 0.0, 1.0, 0.0));
			quatz = glm::normalize(glm::quat(1.0, 0.0, 0.0, 1.0));
			quati = glm::normalize(glm::quat(1.0, 0.0, 0.0, 0.0));

			quatx_stdv = 0.1;
			quaty_stdv = 0.1;
			quatz_stdv = 0.1;
			quatx_stdv_long = 0.2;
			quaty_stdv_long = 1.5;
			quatz_stdv_long = 0.2;

			quat_x = std::uniform_real_distribution<float>(-quatx_stdv, quatx_stdv);
			quat_y = std::uniform_real_distribution<float>(-quaty_stdv, quaty_stdv);
			quat_z = std::uniform_real_distribution<float>(-quatz_stdv, quatz_stdv);
			quat_x_long = std::uniform_real_distribution<float>(-quatx_stdv_long, quatx_stdv_long);
			quat_y_long = std::uniform_real_distribution<float>(-quaty_stdv_long, quaty_stdv_long);
			quat_z_long = std::uniform_real_distribution<float>(-quatz_stdv_long, quatz_stdv_long);
			
			st = 0.2;
			lsr = 0.05;
			ss = 0.01;

			t = std::uniform_real_distribution<float>(-st, st);
			t_dim = std::uniform_real_distribution<float>(-st/4, st/4);
			t_z = std::uniform_real_distribution<float>(-0.2, 0.2);
			lr = std::uniform_real_distribution<float>(-lsr, lsr);;
			sc = std::uniform_real_distribution<float>(-ss, ss);
			toexrot = std::uniform_real_distribution<float>(glm::radians(-15.0f), glm::radians(15.0f));
			legxrot = std::uniform_real_distribution<float>(glm::radians(-20.0f), glm::radians(30.0f));
			legzrot = std::uniform_real_distribution<float>(glm::radians(-30.0f), glm::radians(30.0f));

			std::random_device rd;
			gen = std::mt19937(rd());

			// Get and set up shaders
			RepeatShader = Shader("../res/shaders/PTVS.glsl", "../res/shaders/PTFSRepeat.glsl");
			SubtractionShader = Shader("../res/shaders/SubtractionVertexShader.glsl", "../res/shaders/SubtractionFragmentShader.glsl");
			RTTShader = Shader("../res/shaders/RTTVShader.glsl", "../res/shaders/RTTFShader.glsl");
			R2Shader = Shader("../res/shaders/PassThroughQuadVertexShader.glsl", "../res/shaders/Reduction2FShader.glsl");
			PTShader = Shader("../res/shaders/PTVS.glsl", "../res/shaders/PTFS.glsl");
			ModelShader = Shader("../res/shaders/ModelVS.glsl", "../res/shaders/ModelFS.glsl");
			
			// Load the skeleton and associated bone matrices
			footSkeleton_L = SkeletonModel("../res/eric_foot_left_above_ankle.dae");
			MeshToBoneLeg_L = footSkeleton_L.meshes[0].offsetMatricies[3];
			MeshToBoneToe_L = footSkeleton_L.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_L = glm::inverse(MeshToBoneLeg_L);
			BoneToMeshToe_L = glm::inverse(MeshToBoneToe_L);	

			footSkeleton_R = SkeletonModel("../res/eric_foot_right_above_ankle.dae");
			MeshToBoneLeg_R = footSkeleton_R.meshes[0].offsetMatricies[3];
			MeshToBoneToe_R = footSkeleton_R.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_R = glm::inverse(MeshToBoneLeg_R);
			BoneToMeshToe_R = glm::inverse(MeshToBoneToe_R);	

			// PSO tile offsets
			float translations[NumParticles];
			for (int i = 0; i < NumParticles; i++) translations[i] = i*2.0/NumParticles;

			ConfigureSkeleton(footSkeleton_L, translations, NumParticles, instanceVBO_L, transformationInstanceBuffer_L, rottoeVB_L, rotlegVB_L);
			ConfigureSkeleton(footSkeleton_R, translations, NumParticles, instanceVBO_R, transformationInstanceBuffer_R, rottoeVB_R, rotlegVB_R);

			float quadVertices[] = {
				// positions   // texCoords
				-1.0f,  1.0f,  0.0f, 0.0f,
				-1.0f, -1.0f,  0.0f, 1.0f,
				1.0f, -1.0f,  1.0f, 1.0f,

				-1.0f,  1.0f,  0.0f, 0.0f,
				1.0f, -1.0f,  1.0f, 1.0f,
				1.0f,  1.0f,  1.0f, 0.0f
			};

			float repeatQuadLargeVertices[] = {
				// positions   // texCoords
				-1.0f,  1.0f,  0.0f, 0.0f,
				-1.0f, -1.0f,  0.0f, 1.0f,
				1.0f, -1.0f,  1.0f*NumParticles, 1.0f,

				-1.0f,  1.0f,  0.0f, 0.0f,
				1.0f, -1.0f,  1.0f*NumParticles, 1.0f,
				1.0f,  1.0f,  1.0f*NumParticles, 0.0f
			};

			// declare quad
			glGenVertexArrays(1, &quadVAO);
			glGenBuffers(1, &quadVBO);
			glBindVertexArray(quadVAO);
			glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), INT2VOIDP(0));
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), INT2VOIDP((2*sizeof(float))));

			glBindVertexArray(0);

			// declare large repeat quad
			glGenVertexArrays(1, &repeatQuadLargeVAO);
			glGenBuffers(1, &repeatQuadLargeVBO);
			glBindVertexArray(repeatQuadLargeVAO);
			glBindBuffer(GL_ARRAY_BUFFER, repeatQuadLargeVBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(repeatQuadLargeVertices), &repeatQuadLargeVertices, GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), INT2VOIDP(0));
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), INT2VOIDP(2*sizeof(float)));

			glBindVertexArray(0);

			GenerateTexture(refdepthtex, 0, 32, 32);
			GenerateTextureWithFramebuffer(peng, repeattex, 1, NumParticles*32, 32);
			GenerateTextureWithFramebuffer(ping, depthtexture, 2, NumParticles*32, 32);
			GenerateTextureWithFramebuffer(pong, difftex, 3, NumParticles*32, 32);
			GenerateTextureWithFramebuffer(pling, tex16, 4, NumParticles*16, 16);
			GenerateTextureWithFramebuffer(plang, tex8, 5, NumParticles*8, 8);
			GenerateTextureWithFramebuffer(plong, tex4, 6, NumParticles*4, 4);
			GenerateTextureWithFramebuffer(plung, tex2, 7, NumParticles*2, 2);
			GenerateTextureWithFramebuffer(pleng, tex1, 8, NumParticles, 1);

			// enable depth testing
			glEnable(GL_DEPTH_TEST);
		}

		void SamplePoseParametersSeparateInitial(float xLeft, float xRight, float yTop, float yBottom, float depthAround)
		{
			std::uniform_real_distribution<float> xdelta(xLeft, xRight);
			std::uniform_real_distribution<float> ydelta(yTop, yBottom);
			for (int i = 0; i < NumParticles; i++)
			{
				glm::quat quat = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_long(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				Particles[i] = Particle(param);
			}
		}

		void SamplePoseParametersSeparateIterated(PoseParameters prevpose)
		{
			for (int i = 0; i < NumParticles; i++)
			{
				glm::quat quat = glm::normalize(glm::mix(quati, quatz, quat_z(gen)) * glm::mix(quati, quaty, quat_y(gen)) * glm::mix(quati, quatx, quat_x(gen)) * prevpose.GlobalQuat);
				PoseParameters param = PoseParameters(prevpose.XTranslation + t_dim(gen), prevpose.YTranslation + t_dim(gen), prevpose.ZTranslation + t_dim(gen), quat, prevpose.LegXRot + lr(gen), prevpose.LegZRot + lr(gen), prevpose.LegZRot+ lr(gen), prevpose.Scale + sc(gen));
				Particles[i] = Particle(param);
			}
		}

		EnergyPosePair Run(bool is_left, float* refImg, Box& bbox, int iters, glm::mat4& ProjMat, bool isInitialSample, float depthAround, Intrinsics intrinsics, PoseParameters* currentBestPose) 
		{	
			// change to PSO OpenGL context
			glfwMakeContextCurrent(window);
			glfwHideWindow(window);

			// update reference texture with new crop
			glActiveTexture(GL_TEXTURE0);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 32, 32, GL_DEPTH_COMPONENT, GL_FLOAT, refImg);

			// Get x and y to sample around (image center) based on depth
			float bbox_XLeft = bbox.x + bbox.width/4;
			float bbox_XRight = bbox.x + 3*bbox.width/4;
			float bbox_YTop = bbox.y + bbox.width/4;
			float bbox_YBottom = bbox.y + 3*bbox.width/4;
			float XLeft = depthAround * (intrinsics.ppx - bbox_XLeft) / intrinsics.fx;
			float XRight = depthAround * (intrinsics.ppx - bbox_XRight) / intrinsics.fx;
			float YTop = depthAround * (bbox_YTop - intrinsics.ppy) / intrinsics.fy;
			float YBottom = depthAround * (bbox_YBottom - intrinsics.ppy) / intrinsics.fy;

			// intialize PSO particles
			if (isInitialSample)
			{
				SamplePoseParametersSeparateInitial(XLeft, XRight, YTop, YBottom, depthAround);
			}
			else 
			{
				SamplePoseParametersSeparateIterated(*currentBestPose);
			}

			// Setup finished, start the particle swarm!
			PoseParameters GlobalBestPosition;
			float GlobalBestEnergy = std::numeric_limits<float>::infinity();

			// BEGIN TESTING CODE HERE
			glm::mat4 Movements[NumParticles];
			glm::mat4 ToeRotations[NumParticles];
			glm::mat4 LegRotations[NumParticles];

			for (int i = 0; i < NumParticles; i++)
			{
				PoseParameters currparam = Particles[i].Position;
				// Set up MVP matricies
				glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(currparam.XTranslation, currparam.YTranslation, currparam.ZTranslation));
				model = model * glm::mat4_cast(currparam.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparam.Scale, currparam.Scale, currparam.Scale));
				Movements[i] = model;	
				ToeRotations[i] = glm::eulerAngleXYZ(currparam.ToeXRot, 0.0f, 0.0f);
				LegRotations[i] = glm::eulerAngleXYZ(currparam.LegXRot, 0.0f, currparam.LegZRot);
			}

			if (is_left)
			{
				glNamedBufferSubData(transformationInstanceBuffer_L, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
				glNamedBufferSubData(rottoeVB_L, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
				glNamedBufferSubData(rotlegVB_L, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
			}
			else 
			{
				glNamedBufferSubData(transformationInstanceBuffer_R, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
				glNamedBufferSubData(rottoeVB_R, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
				glNamedBufferSubData(rotlegVB_R, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
			}

			for (int generation = 0; generation < iters; generation++)
			{
				if (generation % 30 == 0)
				{
					for (int i = 0; i < NumParticles; i ++)
					{
						int randIndex = rand() % 32;
						glm::quat quat = glm::normalize(glm::mix(quati, quaty, quat_y_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
						Particles[i].Position.GlobalQuat = quat * Particles[i].Position.GlobalQuat;
					}
				}

				glm::mat4 Movements[NumParticles];
				glm::mat4 ToeRotations[NumParticles];
				glm::mat4 LegRotations[NumParticles];

				for (int i = 0; i < NumParticles; i++)
				{
					PoseParameters currparam = Particles[i].Position;
					// Set up MVP matricies
					glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(currparam.XTranslation, currparam.YTranslation, currparam.ZTranslation));
					model = model * glm::mat4_cast(currparam.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparam.Scale, currparam.Scale, currparam.Scale));
					Movements[i] = model;	
					ToeRotations[i] = glm::eulerAngleXYZ(currparam.ToeXRot, 0.0f, 0.0f);
					LegRotations[i] = glm::eulerAngleXYZ(currparam.LegXRot, 0.0f, currparam.LegZRot);
				}

				if (is_left)
				{
					glNamedBufferSubData(transformationInstanceBuffer_L, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rottoeVB_L, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
					glNamedBufferSubData(rotlegVB_L, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
				}
				else 
				{
					glNamedBufferSubData(transformationInstanceBuffer_R, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rottoeVB_R, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
					glNamedBufferSubData(rotlegVB_R, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
				}

				glViewport(0, 0, NumParticles*32, 32);

				glBindFramebuffer(GL_FRAMEBUFFER, peng);
				glClear(GL_DEPTH_BUFFER_BIT);
				RepeatShader.use();
				RepeatShader.setInt("tex", 0);
				glBindVertexArray(repeatQuadLargeVAO);
				glViewport(0, 0, 32*NumParticles, 32);
				glDrawArrays(GL_TRIANGLES, 0, 6);

				//float reptex[32*32*NumParticles];
				//glGetTextureImage(repeattex, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*NumParticles*32*32, reptex);
				//cv::Mat repteximg = cv::Mat(32*NumParticles, 32, CV_32FC1, reptex).clone();		
				//cv::flip(repteximg, repteximg, 0);
				//cv::imwrite("/home/eric/Dev/DepthImageAnnotator/include/psoout/rep.exr", repteximg);

				glBindFramebuffer(GL_FRAMEBUFFER, ping);
				glClear(GL_DEPTH_BUFFER_BIT);
				RTTShader.use(); 
				RTTShader.setMat4("u_P", ProjMat);
				RTTShader.setVec3("bbox", glm::vec3(bbox.x, 720.0f - bbox.y - bbox.width, bbox.width));
				if (is_left)
				{
					RTTShader.setMat4("m2btoe", MeshToBoneToe_L);
					RTTShader.setMat4("m2bleg", MeshToBoneLeg_L);
					RTTShader.setMat4("b2mtoe", BoneToMeshToe_L);
					RTTShader.setMat4("b2mleg", BoneToMeshLeg_L);
					glBindVertexArray(footSkeleton_L.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, footSkeleton_L.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
				}
				else 
				{
					RTTShader.setMat4("m2btoe", MeshToBoneToe_R);
					RTTShader.setMat4("m2bleg", MeshToBoneLeg_R);
					RTTShader.setMat4("b2mtoe", BoneToMeshToe_R);
					RTTShader.setMat4("b2mleg", BoneToMeshLeg_R);
					glBindVertexArray(footSkeleton_R.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, footSkeleton_R.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
				}

				//float rentex[32*32*NumParticles];
				//glGetTextureImage(depthtexture, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*NumParticles*32*32, rentex);
				//cv::Mat renteximg = cv::Mat(32*NumParticles, 32, CV_32FC1, rentex).clone();		
				//cv::flip(renteximg, renteximg, 0);
				//char renOutPath[150];
				//sprintf(renOutPath, "%s%d%s", "/home/eric/Dev/DepthImageAnnotator/include/psoout/pso_", generation, ".exr");
				//cv::imwrite(renOutPath, renteximg);

				glBindFramebuffer(GL_FRAMEBUFFER, pong);
				glClear(GL_DEPTH_BUFFER_BIT);
				SubtractionShader.use();
				SubtractionShader.setInt("screenTexture", 1);
				SubtractionShader.setInt("gendepTexture", 2);
				glBindVertexArray(quadVAO);
				glDrawArrays(GL_TRIANGLES, 0, 6);
				glViewport(0, 0, NumParticles*32, 32);

				glBindFramebuffer(GL_FRAMEBUFFER, pling);
				glClear(GL_DEPTH_BUFFER_BIT);
				R2Shader.use();
				R2Shader.setInt("tex", 3);
				R2Shader.setFloat("width", NumParticles*32.0f);
				R2Shader.setFloat("height", 32.0f);
				glViewport(0, 0, NumParticles*32, 32);
				glDrawArrays(GL_TRIANGLES, 0 , 6);

				glBindFramebuffer(GL_FRAMEBUFFER, plang);
				glClear(GL_DEPTH_BUFFER_BIT);
				R2Shader.use();
				R2Shader.setInt("tex", 4);
				R2Shader.setFloat("width", NumParticles*16.0f);
				R2Shader.setFloat("height", 16.0f);
				glViewport(0, 0, NumParticles*16, 16);
				glDrawArrays(GL_TRIANGLES, 0 , 6);

				glBindFramebuffer(GL_FRAMEBUFFER, plong);
				glClear(GL_DEPTH_BUFFER_BIT);
				R2Shader.use();
				R2Shader.setInt("tex", 5);
				R2Shader.setFloat("width", NumParticles*8.0f);
				R2Shader.setFloat("height", 8.0f);
				glViewport(0, 0, NumParticles*8, 8);
				glDrawArrays(GL_TRIANGLES, 0 , 6);

				glBindFramebuffer(GL_FRAMEBUFFER, plung);
				glClear(GL_DEPTH_BUFFER_BIT);
				R2Shader.use();
				R2Shader.setInt("tex", 6);
				R2Shader.setFloat("width", NumParticles*4.0f);
				R2Shader.setFloat("height", 4.0f);
				glViewport(0, 0, NumParticles*4, 4);
				glDrawArrays(GL_TRIANGLES, 0 , 6);

				glBindFramebuffer(GL_FRAMEBUFFER, pleng);
				glClear(GL_DEPTH_BUFFER_BIT);
				R2Shader.use();
				R2Shader.setInt("tex", 7);
				R2Shader.setFloat("width", NumParticles*2.0f);
				R2Shader.setFloat("height", 2.0f);
				glViewport(0, 0, NumParticles*2, 2);
				glDrawArrays(GL_TRIANGLES, 0 , 6);

				float currentdt[NumParticles];
				glGetTextureImage(tex1, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*NumParticles, currentdt);
				
				// first loop to update local bests and global best left foot
				for (int p = 0; p < NumParticles; p++)
				{
					float ParticleEnergy = currentdt[p];
					if (ParticleEnergy < Particles[p].BestEnergyScore)
					{
						Particles[p].BestEnergyScore = ParticleEnergy;
						Particles[p].BestPosition = Particles[p].Position;
						if (ParticleEnergy < GlobalBestEnergy)
						{
							GlobalBestEnergy = ParticleEnergy;
							GlobalBestPosition = Particles[p].Position;
						}
					}
				}

				if (generation == iters - 1) break;

				// second loop to update position and velocities left foot
				for (int p = 0; p < NumParticles; p++)
				{
					float r1 = ((float) std::rand() / RAND_MAX);
					float r2 = ((float) std::rand() / RAND_MAX);
					PoseParameters personalVelocity = (Particles[p].BestPosition - Particles[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocity = (GlobalBestPosition - Particles[p].Position)*(SocialConst*r2);
					Particles[p].Velocity = Particles[p].Velocity + (personalVelocity + socialVelocity)*ConstrictionConst;
					Particles[p].Velocity.AssuageVelocity();
					Particles[p].Position = Particles[p].Position + Particles[p].Velocity; 
					Particles[p].Position.AssuagePosition();
				}
				
			}	

			return EnergyPosePair(GlobalBestPosition, GlobalBestEnergy);
		}

		void WriteImage(float* currentdt, int w, int h, PoseParameters params, bool is_left, Intrinsics& intrinsics)
		{
			float scale_x = w / 1280.0f;
			float scale_y = h / 720.0f;
			float scaled_fx = intrinsics.fx * scale_x;
			float scaled_fy = intrinsics.fy * scale_y;
			float scaled_ppx = intrinsics.ppx * scale_x;
			float scaled_ppy = intrinsics.ppy * scale_y;

			float persp_t[16] = 
			{
				scaled_fx,       0,                0,                                    0,
				0,               -scaled_fy,       0,                                    0,
				-scaled_ppx,     -scaled_ppy,      intrinsics.zFar + intrinsics.zNear,  -1,
				0,               0,                intrinsics.zNear*intrinsics.zFar,     0     
			};
			glm::mat4 persp = glm::make_mat4(persp_t);
			glm::mat4 ortho = glm::ortho(intrinsics.left, (float)w, (float)h, intrinsics.top, intrinsics.zNear, intrinsics.zFar);
			glm::mat4 projection = ortho*persp;

			glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(params.XTranslation, params.YTranslation, params.ZTranslation));
			model = model * glm::mat4_cast(params.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(params.Scale, params.Scale, params.Scale));
			glm::mat4 toerotMatrix = glm::eulerAngleXYZ(params.ToeXRot, 0.0f, 0.0f);
			glm::mat4 legrotMatrix = glm::eulerAngleXYZ(params.LegXRot, 0.0f, params.LegZRot);

			// set up framebuffer with specific width and height
			GenerateTextureWithFramebuffer(picholder, pictex, 10, w, h);

			glBindFramebuffer(GL_FRAMEBUFFER, picholder);
			glClear(GL_DEPTH_BUFFER_BIT);
			ModelShader.use();
			ModelShader.setMat4("model", model);
			ModelShader.setMat4("projection", projection);
			ModelShader.setMat4("toerotMatrix", toerotMatrix);
			ModelShader.setMat4("legrotMatrix", legrotMatrix);

			if (is_left)
			{
				ModelShader.setMat4("m2btoe", MeshToBoneToe_L);
				ModelShader.setMat4("m2bleg", MeshToBoneLeg_L);
				ModelShader.setMat4("b2mtoe", BoneToMeshToe_L);
				ModelShader.setMat4("b2mleg", BoneToMeshLeg_L);
				glBindVertexArray(footSkeleton_L.meshes[0].VAO);
				glEnable(GL_DEPTH_TEST);
				glViewport(0, 0, w, h);
				glDrawElements(GL_TRIANGLES, footSkeleton_L.meshes[0].indices.size(), GL_UNSIGNED_INT, 0);
			}
			else 
			{
				ModelShader.setMat4("m2btoe", MeshToBoneToe_R);
				ModelShader.setMat4("m2bleg", MeshToBoneLeg_R);
				ModelShader.setMat4("b2mtoe", BoneToMeshToe_R);
				ModelShader.setMat4("b2mleg", BoneToMeshLeg_R);
				glBindVertexArray(footSkeleton_R.meshes[0].VAO);
				glEnable(GL_DEPTH_TEST);
				glViewport(0, 0, w, h);
				glDrawElements(GL_TRIANGLES, footSkeleton_R.meshes[0].indices.size(), GL_UNSIGNED_INT, 0);
			}

			glGetTextureImage(pictex, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*w*h, currentdt);
			cv::Mat depth_image(h, w, CV_32FC1, &currentdt[0]);
			cv::flip(depth_image, depth_image, 0);	
			//cv::imwrite("/home/eric/Dev/DepthImageAnnotator/res/out_depth_image.exr", depth_image);
		}
};

class DepthImageAnnotator
{
	private:
		Reinitializer reinit;
		PSO pso;
		
	public:
		DepthImageAnnotator() {}
	
	public:
		PoseParameters FindSolution(float* depth_data, int n, int w, int h, bool is_left, Box& bbox, Intrinsics& intrinsics, int iterations, int initial_samples, int iterated_samples)
		{
			cv::Mat depth_image = cv::Mat(w, h, CV_32FC1, depth_data).clone();

			float persp_t[16] = 
			{
				intrinsics.fx,   0,                0,                                    0,
				0,               -intrinsics.fy,   0,                                    0,
				-intrinsics.ppx, -intrinsics.ppy,  intrinsics.zFar + intrinsics.zNear,  -1,
				0,               0,                intrinsics.zNear*intrinsics.zFar,     0     
			};
			glm::mat4 persp = glm::make_mat4(persp_t);
			glm::mat4 ortho = glm::ortho(intrinsics.left, intrinsics.right, intrinsics.bottom, intrinsics.top, intrinsics.zNear, intrinsics.zFar);
			glm::mat4 projection = ortho*persp;

			const int bins = 1000;
			int histSize[] = {bins};
			float range[] = { 0.f, 10.f };
			const float* ranges[] = { range };
			int channels[] = {0};
			cv::MatND hist;
			cv::calcHist( &depth_image, 1, channels, depth_image!=0, // do not use mask
							 hist, 1, histSize, ranges,
							 true, // the histogram is uniform
							 false );

			cv::Point max_idx;
			cv::minMaxLoc(hist , 0, 0, 0, &max_idx);
			// assume this is the correct mode
			float mode = max_idx.y * (10.f / bins);

			depth_image.setTo(3.0f, depth_image == 0.0f);
			depth_image = depth_image / 3.0f;
		
			cv::Mat_<float> cropped_and_resized32 = cv::Mat_<float>::zeros(32, 32);
			cv::resize(depth_image, cropped_and_resized32, cv::Size(32, 32), 0, 0);
			float* df_arr = (float*)cropped_and_resized32.data;
		
			float BestEnergy = std::numeric_limits<float>::infinity();
			PoseParameters BestPosition = PoseParameters();
			for (int i = 0; i < initial_samples; i++)
			{
				EnergyPosePair epp = pso.Run(is_left, df_arr, bbox, iterations, projection, true, -mode, intrinsics, nullptr);
				if (epp.energy < BestEnergy)
				{
					BestEnergy = epp.energy;
					BestPosition = epp.position;
				}
			}
			for (int i = 0; i < iterated_samples; i++)
			{
				EnergyPosePair epp = pso.Run(is_left, df_arr, bbox, iterations, projection, false, -mode, intrinsics, &epp.position);
				if (epp.energy < BestEnergy)
				{
					BestEnergy = epp.energy;
					BestPosition = epp.position;
				}
			}

			return BestPosition;
		}

		void WriteImage(float* currentdt, int n, int w, int h, PoseParameters params, bool is_left, Intrinsics& intrinsics)
		{
			pso.WriteImage(currentdt, w, h, params, is_left, intrinsics);
		}
};
