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
#include <fstream>
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
			float angleIncrement = glm::radians(10.0);
			float localAngleIncrement = glm::radians(3.0);
			float scaleIncrement = 0.01;
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

struct FiveSet
{
	PoseParameters FirstBest;
	PoseParameters SecondBest;
	PoseParameters ThirdBest;
	PoseParameters FourthBest;
	PoseParameters FifthBest;

	FiveSet(PoseParameters first, PoseParameters second, PoseParameters third, PoseParameters fourth, PoseParameters fifth) : FirstBest{first}, SecondBest{second}, ThirdBest{third}, FourthBest{fourth}, FifthBest{fifth} {}
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
		std::vector<Particle> ParticlesRight1, ParticlesRight2, ParticlesFront1, ParticlesFront2, ParticlesLeft1, ParticlesLeft2, ParticlesBack1, ParticlesBack2;
		float CognitiveConst, SocialConst, ConstrictionConst; // PSO population constants
		// OpenGL vars
		GLFWwindow* window;
		glm::mat4 MeshToBoneToe_L, MeshToBoneLeg_L, BoneToMeshToe_L, BoneToMeshLeg_L; // bone transformation matricies left foot
		glm::mat4 MeshToBoneToe_R, MeshToBoneLeg_R, BoneToMeshToe_R, BoneToMeshLeg_R; // bone transformation matricies right foot
		glm::mat4 MeshToBoneLeg_L_Pole, BoneToMeshLeg_L_Pole;
		glm::mat4 MeshToBoneLeg_R_Pole, BoneToMeshLeg_R_Pole;
		Shader RepeatShader, SubtractionShader, RTTShader, PoleShader, R2Shader, PTShader, ModelShader, PoleWriteShader;
		SkeletonModel footSkeleton_L;
		SkeletonModel footSkeleton_R;
		SkeletonModel pole_L;
		SkeletonModel pole_R;
		// quads, textures, and buffers
		GLuint quadVAO, quadVBO, repeatQuadLargeVAO, repeatQuadLargeVBO, refdepthtex, peng, repeattex, ping, depthtexture, pong, difftex, pling, tex16, plang, tex8, plong, tex4, plung, tex2, pleng, tex1;
		// picture
		GLuint picholder, pictex;
		// instance buffers left foot
		GLuint instanceVBO_L, transformationInstanceBuffer_L, rottoeVB_L, rotlegVB_L;
		// instance buffers right foot
		GLuint instanceVBO_R, transformationInstanceBuffer_R, rottoeVB_R, rotlegVB_R;
		// instance buffers left pole
		GLuint instanceVBO_L_Pole, transformationInstanceBuffer_L_Pole, rotlegVB_L_Pole;
		GLuint instanceVBO_R_Pole, transformationInstanceBuffer_R_Pole, rotlegVB_R_Pole;
		// instance buffers right pole
		// sampling variables
		glm::quat quatx, quaty, quatz, quati; // standard quaternions representing 90 degree rotations
		float quatx_stdv, quaty_stdv, quatz_stdv; // standard deviations for uniform quaternion sampling
		float quatx_stdv_long, quaty_stdv_long, quatz_stdv_long; // standard deviations for uniform quaternion sampling long
		float quaty_stdv_shuffle;
		std::uniform_real_distribution<float> quat_x, quat_y, quat_z; // uniform quaternion distributions
		std::uniform_real_distribution<float> quat_x_long, quat_y_long, quat_z_long;
		std::uniform_real_distribution<float> quat_y_right1, quat_y_right2, quat_y_front1, quat_y_front2, quat_y_left1, quat_y_left2, quat_y_back1, quat_y_back2;
		std::uniform_real_distribution<float> quat_y_shuffle;
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

		static void ConfigurePole(SkeletonModel& model, float* translations, int numInstances, GLuint& instanceVBO, GLuint& transformationInstanceBuffer, GLuint& rotlegVB) 
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
		PSO(float CogConst=2.8, float SocConst=1.3) : 
			NumParticles{256}, // note this must divide into 1280 evenly	
			CognitiveConst{CogConst}, 
			SocialConst{SocConst}, 
			ConstrictionConst{0.0f}, 
			window{nullptr}
		{
			// Set the initial size of the Particle vector
			ParticlesRight1.resize(32);
			ParticlesRight2.resize(32);
			ParticlesFront1.resize(32);
			ParticlesFront2.resize(32);
			ParticlesLeft1.resize(32);
			ParticlesLeft2.resize(32);
			ParticlesBack1.resize(32);
			ParticlesBack2.resize(32);

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
			quaty_stdv_long = 2.0;
			quatz_stdv_long = 0.9;
			quaty_stdv_shuffle = 1.5;

			quat_x = std::uniform_real_distribution<float>(-quatx_stdv, quatx_stdv);
			quat_y = std::uniform_real_distribution<float>(-quaty_stdv, quaty_stdv);
			quat_z = std::uniform_real_distribution<float>(-quatz_stdv, quatz_stdv);
			quat_x_long = std::uniform_real_distribution<float>(-quatx_stdv_long, quatx_stdv_long);
			quat_y_long = std::uniform_real_distribution<float>(-quaty_stdv_long, quaty_stdv_long);
			quat_z_long = std::uniform_real_distribution<float>(-quatz_stdv_long, quatz_stdv_long);
			quat_y_shuffle = std::uniform_real_distribution<float>(-quaty_stdv_shuffle, quaty_stdv_shuffle);
			quat_y_right1 = std::uniform_real_distribution<float>(0.0, 0.5);
			quat_y_right2 = std::uniform_real_distribution<float>(0.5, 1.0);
			quat_y_front1 = std::uniform_real_distribution<float>(1.0, 1.5);
			quat_y_front2 = std::uniform_real_distribution<float>(1.5, 2.0);
			quat_y_left1 = std::uniform_real_distribution<float>(2.0, 2.5);
			quat_y_left2 = std::uniform_real_distribution<float>(2.5, 3.0);
			quat_y_back1 = std::uniform_real_distribution<float>(3.0, 3.5);
			quat_y_back2= std::uniform_real_distribution<float>(3.5, 4.0);
			
			st = 0.2;
			lsr = 0.05;
			ss = 0.01;

			t = std::uniform_real_distribution<float>(-st, st);
			t_dim = std::uniform_real_distribution<float>(-st/4, st/4);
			t_z = std::uniform_real_distribution<float>(-0.5, 0.5);
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
			PoleShader = Shader("../res/shaders/PoleVShader.glsl", "../res/shaders/PoleFShader.glsl");
			R2Shader = Shader("../res/shaders/PassThroughQuadVertexShader.glsl", "../res/shaders/Reduction2FShader.glsl");
			PTShader = Shader("../res/shaders/PTVS.glsl", "../res/shaders/PTFS.glsl");
			ModelShader = Shader("../res/shaders/ModelVS.glsl", "../res/shaders/ModelFS.glsl");
			PoleWriteShader = Shader("../res/shaders/PoleWriteVShader.glsl", "../res/shaders/PoleWriteFShader.glsl");
			
			// Load the skeleton and associated bone matrices
			footSkeleton_L = SkeletonModel("../res/eric_foot_left_even_shorter.dae");
			MeshToBoneLeg_L = footSkeleton_L.meshes[0].offsetMatricies[3];
			MeshToBoneToe_L = footSkeleton_L.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_L = glm::inverse(MeshToBoneLeg_L);
			BoneToMeshToe_L = glm::inverse(MeshToBoneToe_L);	

			footSkeleton_R = SkeletonModel("../res/eric_foot_right_even_shorter.dae");
			MeshToBoneLeg_R = footSkeleton_R.meshes[0].offsetMatricies[3];
			MeshToBoneToe_R = footSkeleton_R.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_R = glm::inverse(MeshToBoneLeg_R);
			BoneToMeshToe_R = glm::inverse(MeshToBoneToe_R);	

			// Load pole used to account for occlusion
			pole_L = SkeletonModel("../res/eric_leg_left.dae");
			MeshToBoneLeg_L_Pole = pole_L.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_L_Pole = glm::inverse(MeshToBoneLeg_L_Pole);
			
			pole_R = SkeletonModel("../res/eric_leg_right.dae");
			MeshToBoneLeg_R_Pole = pole_R.meshes[0].offsetMatricies[2];
			BoneToMeshLeg_R_Pole = glm::inverse(MeshToBoneLeg_R_Pole);

			// PSO tile offsets
			float translations[NumParticles];
			for (int i = 0; i < NumParticles; i++) translations[i] = i*2.0/NumParticles;

			ConfigureSkeleton(footSkeleton_L, translations, NumParticles, instanceVBO_L, transformationInstanceBuffer_L, rottoeVB_L, rotlegVB_L);
			ConfigureSkeleton(footSkeleton_R, translations, NumParticles, instanceVBO_R, transformationInstanceBuffer_R, rottoeVB_R, rotlegVB_R);
			ConfigurePole(pole_L, translations, NumParticles, instanceVBO_L_Pole, transformationInstanceBuffer_L_Pole, rotlegVB_L_Pole);
			ConfigurePole(pole_R, translations, NumParticles, instanceVBO_R_Pole, transformationInstanceBuffer_R_Pole, rotlegVB_R_Pole);

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
			for (int i = 0; i < 32; i++)
			{
				glm::quat quat_right1 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_right1(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_right1 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_right1, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesRight1[i] = Particle(param_right1);

				glm::quat quat_right2 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_right2(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_right2 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_right2, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesRight2[i] = Particle(param_right2);

				glm::quat quat_front1 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_front1(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_front1 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_front1, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesFront1[i] = Particle(param_front1);

				glm::quat quat_front2 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_front2(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_front2 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_front2, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesFront2[i] = Particle(param_front2);

				glm::quat quat_left1 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_left1(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_left1 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_left1, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesLeft1[i] = Particle(param_left1);

				glm::quat quat_left2 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_left2(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_left2 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_left2, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesLeft2[i] = Particle(param_left2);

				glm::quat quat_back1 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_back1(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_back1 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_back1, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesBack1[i] = Particle(param_back1);

				glm::quat quat_back2 = glm::normalize(glm::mix(quati, quatz, quat_z_long(gen)) * glm::mix(quati, quaty, quat_y_back2(gen)) * glm::mix(quati, quatx, quat_x_long(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				PoseParameters param_back2 = PoseParameters(xdelta(gen), ydelta(gen), depthAround + t_z(gen), quat_back2, toexrot(gen), legxrot(gen), legzrot(gen), 1.0f + sc(gen));
				ParticlesBack2[i] = Particle(param_back2);
			}
		}

		FiveSet Run(bool is_left, float* refImg, Box& bbox, int iters, glm::mat4& ProjMat, float depthAround, Intrinsics intrinsics) 
		{	
			// change to PSO OpenGL context
			glfwMakeContextCurrent(window);
			glfwHideWindow(window);

			// update reference texture with new crop
			glActiveTexture(GL_TEXTURE0);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 32, 32, GL_DEPTH_COMPONENT, GL_FLOAT, refImg);

			// Get x and y to sample around (image center) based on depth
			float bbox_XLeft = bbox.x + bbox.width/5;
			float bbox_XRight = bbox.x + 4*bbox.width/5;
			float bbox_YTop = bbox.y + bbox.width/5;
			float bbox_YBottom = bbox.y + 4*bbox.width/5;
			float XLeft = depthAround * (intrinsics.ppx - bbox_XLeft) / intrinsics.fx;
			float XRight = depthAround * (intrinsics.ppx - bbox_XRight) / intrinsics.fx;
			float YTop = depthAround * (bbox_YTop - intrinsics.ppy) / intrinsics.fy;
			float YBottom = depthAround * (bbox_YBottom - intrinsics.ppy) / intrinsics.fy;

			// intialize PSO particles
			SamplePoseParametersSeparateInitial(XLeft, XRight, YTop, YBottom, depthAround);

			// Setup finished, start the particle swarm!
			PoseParameters GlobalBestPositionRight1, GlobalBestPositionRight2, GlobalBestPositionFront1, GlobalBestPositionFront2, GlobalBestPositionLeft1, GlobalBestPositionLeft2, GlobalBestPositionBack1, GlobalBestPositionBack2;
			float GlobalBestEnergyRight1 = std::numeric_limits<float>::infinity();
			float GlobalBestEnergyRight2 = std::numeric_limits<float>::infinity();
			float GlobalBestEnergyFront1 = std::numeric_limits<float>::infinity();
			float GlobalBestEnergyFront2 = std::numeric_limits<float>::infinity();
			float GlobalBestEnergyLeft1	= std::numeric_limits<float>::infinity();
			float GlobalBestEnergyLeft2	= std::numeric_limits<float>::infinity();
			float GlobalBestEnergyBack1	= std::numeric_limits<float>::infinity();
			float GlobalBestEnergyBack2	= std::numeric_limits<float>::infinity();

			for (int generation = 0; generation < iters; generation++)
			{
				//if (generation % 3 == 0)
				//{
				//	for (int i = 0; i < NumParticles; i ++)
				//	{
				//		int randIndex = rand() % 32;
				//		glm::quat quat = glm::normalize(glm::mix(quati, quaty, quat_y_shuffle(gen)) * glm::quat(1.0, 0.0, 0.0, 0.0));
				//		Particles[i].Position.GlobalQuat = quat * Particles[i].Position.GlobalQuat;
				//	}
				//}

				glm::mat4 Movements[NumParticles];
				glm::mat4 ToeRotations[NumParticles];
				glm::mat4 LegRotations[NumParticles];

				for (int i = 0; i < 32; i++)
				{
					PoseParameters currparamRight1 = ParticlesRight1[i].Position;
					glm::mat4 modelRight1 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamRight1.XTranslation, currparamRight1.YTranslation, currparamRight1.ZTranslation));
					modelRight1 = modelRight1 * glm::mat4_cast(currparamRight1.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamRight1.Scale, currparamRight1.Scale, currparamRight1.Scale));
					Movements[i] = modelRight1;	
					ToeRotations[i] = glm::eulerAngleXYZ(currparamRight1.ToeXRot, 0.0f, 0.0f);
					LegRotations[i] = glm::eulerAngleXYZ(currparamRight1.LegXRot, 0.0f, currparamRight1.LegZRot);

					PoseParameters currparamRight2 = ParticlesRight2[i].Position;
					glm::mat4 modelRight2 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamRight2.XTranslation, currparamRight2.YTranslation, currparamRight2.ZTranslation));
					modelRight2 = modelRight2 * glm::mat4_cast(currparamRight2.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamRight2.Scale, currparamRight2.Scale, currparamRight2.Scale));
					Movements[i + 32] = modelRight2;	
					ToeRotations[i + 32] = glm::eulerAngleXYZ(currparamRight2.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 32] = glm::eulerAngleXYZ(currparamRight2.LegXRot, 0.0f, currparamRight2.LegZRot);

					PoseParameters currparamFront1 = ParticlesFront1[i].Position;
					glm::mat4 modelFront1 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamFront1.XTranslation, currparamFront1.YTranslation, currparamFront1.ZTranslation));
					modelFront1 = modelFront1 * glm::mat4_cast(currparamFront1.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamFront1.Scale, currparamFront1.Scale, currparamFront1.Scale));
					Movements[i + 64] = modelFront1;	
					ToeRotations[i + 64] = glm::eulerAngleXYZ(currparamFront1.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 64] = glm::eulerAngleXYZ(currparamFront1.LegXRot, 0.0f, currparamFront1.LegZRot);

					PoseParameters currparamFront2 = ParticlesFront2[i].Position;
					glm::mat4 modelFront2 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamFront2.XTranslation, currparamFront2.YTranslation, currparamFront2.ZTranslation));
					modelFront2 = modelFront2 * glm::mat4_cast(currparamFront2.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamFront2.Scale, currparamFront2.Scale, currparamFront2.Scale));
					Movements[i + 96] = modelFront2;	
					ToeRotations[i + 96] = glm::eulerAngleXYZ(currparamFront2.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 96] = glm::eulerAngleXYZ(currparamFront2.LegXRot, 0.0f, currparamFront2.LegZRot);

					PoseParameters currparamLeft1 = ParticlesLeft1[i].Position;
					glm::mat4 modelLeft1 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamLeft1.XTranslation, currparamLeft1.YTranslation, currparamLeft1.ZTranslation));
					modelLeft1 = modelLeft1 * glm::mat4_cast(currparamLeft1.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamLeft1.Scale, currparamLeft1.Scale, currparamLeft1.Scale));
					Movements[i + 128] = modelLeft1;	
					ToeRotations[i + 128] = glm::eulerAngleXYZ(currparamLeft1.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 128] = glm::eulerAngleXYZ(currparamLeft1.LegXRot, 0.0f, currparamLeft1.LegZRot);

					PoseParameters currparamLeft2 = ParticlesLeft2[i].Position;
					glm::mat4 modelLeft2 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamLeft2.XTranslation, currparamLeft2.YTranslation, currparamLeft2.ZTranslation));
					modelLeft2 = modelLeft2 * glm::mat4_cast(currparamLeft2.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamLeft2.Scale, currparamLeft2.Scale, currparamLeft2.Scale));
					Movements[i + 160] = modelLeft2;	
					ToeRotations[i + 160] = glm::eulerAngleXYZ(currparamLeft2.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 160] = glm::eulerAngleXYZ(currparamLeft2.LegXRot, 0.0f, currparamLeft2.LegZRot);

					PoseParameters currparamBack2 = ParticlesBack2[i].Position;
					glm::mat4 modelBack2 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamBack2.XTranslation, currparamBack2.YTranslation, currparamBack2.ZTranslation));
					modelBack2 = modelBack2 * glm::mat4_cast(currparamBack2.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamBack2.Scale, currparamBack2.Scale, currparamBack2.Scale));
					Movements[i + 192] = modelBack2;	
					ToeRotations[i + 192] = glm::eulerAngleXYZ(currparamBack2.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 192] = glm::eulerAngleXYZ(currparamBack2.LegXRot, 0.0f, currparamBack2.LegZRot);

					PoseParameters currparamBack1 = ParticlesBack1[i].Position;
					glm::mat4 modelBack1 = glm::translate(glm::mat4(1.0f), glm::vec3(currparamBack1.XTranslation, currparamBack1.YTranslation, currparamBack1.ZTranslation));
					modelBack1 = modelBack1 * glm::mat4_cast(currparamBack1.GlobalQuat) * glm::scale(glm::mat4(1.0f), glm::vec3(currparamBack1.Scale, currparamBack1.Scale, currparamBack1.Scale));
					Movements[i + 224] = modelBack1;	
					ToeRotations[i + 224] = glm::eulerAngleXYZ(currparamBack1.ToeXRot, 0.0f, 0.0f);
					LegRotations[i + 224] = glm::eulerAngleXYZ(currparamBack1.LegXRot, 0.0f, currparamBack1.LegZRot);
				}

				if (is_left)
				{
					glNamedBufferSubData(transformationInstanceBuffer_L, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rottoeVB_L, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
					glNamedBufferSubData(rotlegVB_L, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
					glNamedBufferSubData(transformationInstanceBuffer_L_Pole, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rotlegVB_L_Pole, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
				}
				else 
				{
					glNamedBufferSubData(transformationInstanceBuffer_R, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rottoeVB_R, 0, sizeof(glm::mat4)*NumParticles, &ToeRotations[0]);
					glNamedBufferSubData(rotlegVB_R, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
					glNamedBufferSubData(transformationInstanceBuffer_R_Pole, 0, sizeof(glm::mat4)*NumParticles, &Movements[0]);
					glNamedBufferSubData(rotlegVB_R_Pole, 0, sizeof(glm::mat4)*NumParticles, &LegRotations[0]);
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
				//cv::imwrite("/home/eric/Dev/DepthImageAnnotator/include/psoout/rep.exr", repteximg);

				glBindFramebuffer(GL_FRAMEBUFFER, ping);
				glClear(GL_DEPTH_BUFFER_BIT);
				if (is_left)
				{
					RTTShader.use(); 
					RTTShader.setMat4("u_P", ProjMat);
					RTTShader.setVec3("bbox", glm::vec3(bbox.x, 720.0f - bbox.y - bbox.width, bbox.width));
					RTTShader.setMat4("m2btoe", MeshToBoneToe_L);
					RTTShader.setMat4("m2bleg", MeshToBoneLeg_L);
					RTTShader.setMat4("b2mtoe", BoneToMeshToe_L);
					RTTShader.setMat4("b2mleg", BoneToMeshLeg_L);
					glBindVertexArray(footSkeleton_L.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glDepthFunc(GL_LESS);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, footSkeleton_L.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
					PoleShader.use();
					PoleShader.setMat4("u_P", ProjMat);	
					PoleShader.setVec3("bbox", glm::vec3(bbox.x, 720.0f - bbox.y - bbox.width, bbox.width));
					PoleShader.setMat4("m2bleg", MeshToBoneLeg_L_Pole);
					PoleShader.setMat4("b2mleg", BoneToMeshLeg_L_Pole);
					glBindVertexArray(pole_L.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glDepthFunc(GL_GREATER);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, pole_L.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
				}
				else 
				{
					RTTShader.use(); 
					RTTShader.setMat4("u_P", ProjMat);
					RTTShader.setVec3("bbox", glm::vec3(bbox.x, 720.0f - bbox.y - bbox.width, bbox.width));
					RTTShader.setMat4("m2btoe", MeshToBoneToe_R);
					RTTShader.setMat4("m2bleg", MeshToBoneLeg_R);
					RTTShader.setMat4("b2mtoe", BoneToMeshToe_R);
					RTTShader.setMat4("b2mleg", BoneToMeshLeg_R);
					glBindVertexArray(footSkeleton_R.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glDepthFunc(GL_LESS);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, footSkeleton_R.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
					PoleShader.use();
					PoleShader.setMat4("u_P", ProjMat);	
					PoleShader.setVec3("bbox", glm::vec3(bbox.x, 720.0f - bbox.y - bbox.width, bbox.width));
					PoleShader.setMat4("m2bleg", MeshToBoneLeg_R_Pole);
					PoleShader.setMat4("b2mleg", BoneToMeshLeg_R_Pole);
					glBindVertexArray(pole_R.meshes[0].VAO);
					glEnable(GL_DEPTH_TEST);
					glDepthFunc(GL_GREATER);
					glViewport(0, 0, 32*NumParticles, 32);
					glDrawElementsInstanced(GL_TRIANGLES, pole_R.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, NumParticles);
				}

				//float rentex[32*32*NumParticles];
				//glGetTextureImage(depthtexture, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*NumParticles*32*32, rentex);
				//cv::Mat renteximg = cv::Mat(32*NumParticles, 32, CV_32FC1, rentex).clone();		
				//char renOutPath[150];
				//sprintf(renOutPath, "%s%d%s", "/home/eric/Dev/DepthImageAnnotator/include/psoout/pso_", generation, ".exr");
				//cv::imwrite(renOutPath, renteximg);

				glEnable(GL_DEPTH_TEST);
				glDepthFunc(GL_LESS);
				glBindFramebuffer(GL_FRAMEBUFFER, pong);
				glClear(GL_DEPTH_BUFFER_BIT);
				SubtractionShader.use();
				SubtractionShader.setInt("screenTexture", 1);
				SubtractionShader.setInt("gendepTexture", 2);
				glBindVertexArray(quadVAO);
				glDrawArrays(GL_TRIANGLES, 0, 6);
				glViewport(0, 0, NumParticles*32, 32);

				//float subtex[32*32*NumParticles];
				//glGetTextureImage(difftex, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sizeof(float)*NumParticles*32*32, subtex);
				//cv::Mat subteximg = cv::Mat(32*NumParticles, 32, CV_32FC1, subtex).clone();		
				//char subOutPath[150];
				//sprintf(subOutPath, "%s%d%s", "/home/eric/Dev/DepthImageAnnotator/include/psoout/sub_", generation, ".exr");
				//cv::imwrite(subOutPath, subteximg);

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
				
				// first loop to update local bests and global best
				for (int p = 0; p < 32; p++)
				{
					float ParticleEnergyRight1 = currentdt[p];
					float ParticleEnergyRight2 = currentdt[p + 32];
					float ParticleEnergyFront1 = currentdt[p + 64];
					float ParticleEnergyFront2 = currentdt[p + 96];
					float ParticleEnergyLeft1 = currentdt[p + 128];
					float ParticleEnergyLeft2 = currentdt[p + 160];
					float ParticleEnergyBack1 = currentdt[p + 192];
					float ParticleEnergyBack2 = currentdt[p + 224];

					if (ParticleEnergyRight1 < ParticlesRight1[p].BestEnergyScore)
					{
						ParticlesRight1[p].BestEnergyScore = ParticleEnergyRight1;
						ParticlesRight1[p].BestPosition = ParticlesRight1[p].Position;
						if (ParticleEnergyRight1 < GlobalBestEnergyRight1)
						{
							GlobalBestEnergyRight1 = ParticleEnergyRight1;
							GlobalBestPositionRight1 = ParticlesRight1[p].Position;
						}
					}

					if (ParticleEnergyRight2 < ParticlesRight2[p].BestEnergyScore)
					{
						ParticlesRight2[p].BestEnergyScore = ParticleEnergyRight2;
						ParticlesRight2[p].BestPosition = ParticlesRight2[p].Position;
						if (ParticleEnergyRight2 < GlobalBestEnergyRight2)
						{
							GlobalBestEnergyRight2 = ParticleEnergyRight2;
							GlobalBestPositionRight2 = ParticlesRight2[p].Position;
						}
					}

					if (ParticleEnergyFront1 < ParticlesFront1[p].BestEnergyScore)
					{
						ParticlesFront1[p].BestEnergyScore = ParticleEnergyFront1;
						ParticlesFront1[p].BestPosition = ParticlesFront1[p].Position;
						if (ParticleEnergyFront1 < GlobalBestEnergyFront1)
						{
							GlobalBestEnergyFront1 = ParticleEnergyFront1;
							GlobalBestPositionFront1 = ParticlesFront1[p].Position;
						}
					}

					if (ParticleEnergyFront2 < ParticlesFront2[p].BestEnergyScore)
					{
						ParticlesFront2[p].BestEnergyScore = ParticleEnergyFront2;
						ParticlesFront2[p].BestPosition = ParticlesFront2[p].Position;
						if (ParticleEnergyFront2 < GlobalBestEnergyFront2)
						{
							GlobalBestEnergyFront2 = ParticleEnergyFront2;
							GlobalBestPositionFront2 = ParticlesFront2[p].Position;
						}
					}

					if (ParticleEnergyLeft1 < ParticlesLeft1[p].BestEnergyScore)
					{
						ParticlesLeft1[p].BestEnergyScore = ParticleEnergyLeft1;
						ParticlesLeft1[p].BestPosition = ParticlesLeft1[p].Position;
						if (ParticleEnergyLeft1 < GlobalBestEnergyLeft1)
						{
							GlobalBestEnergyLeft1 = ParticleEnergyLeft1;
							GlobalBestPositionLeft1 = ParticlesLeft1[p].Position;
						}
					}

					if (ParticleEnergyLeft2 < ParticlesLeft2[p].BestEnergyScore)
					{
						ParticlesLeft2[p].BestEnergyScore = ParticleEnergyLeft2;
						ParticlesLeft2[p].BestPosition = ParticlesLeft2[p].Position;
						if (ParticleEnergyLeft2 < GlobalBestEnergyLeft2)
						{
							GlobalBestEnergyLeft2 = ParticleEnergyLeft2;
							GlobalBestPositionLeft2 = ParticlesLeft2[p].Position;
						}
					}

					if (ParticleEnergyBack1 < ParticlesBack1[p].BestEnergyScore)
					{
						ParticlesBack1[p].BestEnergyScore = ParticleEnergyBack1;
						ParticlesBack1[p].BestPosition = ParticlesBack1[p].Position;
						if (ParticleEnergyBack1 < GlobalBestEnergyBack1)
						{
							GlobalBestEnergyBack1 = ParticleEnergyBack1;
							GlobalBestPositionBack1 = ParticlesBack1[p].Position;
						}
					}

					if (ParticleEnergyBack2 < ParticlesBack2[p].BestEnergyScore)
					{
						ParticlesBack2[p].BestEnergyScore = ParticleEnergyBack2;
						ParticlesBack2[p].BestPosition = ParticlesBack2[p].Position;
						if (ParticleEnergyBack2 < GlobalBestEnergyBack2)
						{
							GlobalBestEnergyBack2 = ParticleEnergyBack2;
							GlobalBestPositionBack2 = ParticlesBack2[p].Position;
						}
					}
				}

				if (generation == iters - 1) break;

				// second loop to update position and velocities left foot
				for (int p = 0; p < 32; p++)
				{
					float r1 = ((float) std::rand() / RAND_MAX);
					float r2 = ((float) std::rand() / RAND_MAX);

					PoseParameters personalVelocityRight1 = (ParticlesRight1[p].BestPosition - ParticlesRight1[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityRight1 = (GlobalBestPositionRight1 - ParticlesRight1[p].Position)*(SocialConst*r2);
					ParticlesRight1[p].Velocity = ParticlesRight1[p].Velocity + (personalVelocityRight1 + socialVelocityRight1)*ConstrictionConst;
					ParticlesRight1[p].Velocity.AssuageVelocity();
					ParticlesRight1[p].Position = ParticlesRight1[p].Position + ParticlesRight1[p].Velocity; 
					ParticlesRight1[p].Position.AssuagePosition();

					PoseParameters personalVelocityRight2 = (ParticlesRight2[p].BestPosition - ParticlesRight2[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityRight2 = (GlobalBestPositionRight2 - ParticlesRight2[p].Position)*(SocialConst*r2);
					ParticlesRight2[p].Velocity = ParticlesRight2[p].Velocity + (personalVelocityRight2 + socialVelocityRight2)*ConstrictionConst;
					ParticlesRight2[p].Velocity.AssuageVelocity();
					ParticlesRight2[p].Position = ParticlesRight2[p].Position + ParticlesRight2[p].Velocity; 
					ParticlesRight2[p].Position.AssuagePosition();

					PoseParameters personalVelocityFront1 = (ParticlesFront1[p].BestPosition - ParticlesFront1[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityFront1 = (GlobalBestPositionFront1 - ParticlesFront1[p].Position)*(SocialConst*r2);
					ParticlesFront1[p].Velocity = ParticlesFront1[p].Velocity + (personalVelocityFront1 + socialVelocityFront1)*ConstrictionConst;
					ParticlesFront1[p].Velocity.AssuageVelocity();
					ParticlesFront1[p].Position = ParticlesFront1[p].Position + ParticlesFront1[p].Velocity; 
					ParticlesFront1[p].Position.AssuagePosition();

					PoseParameters personalVelocityFront2 = (ParticlesFront2[p].BestPosition - ParticlesFront2[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityFront2 = (GlobalBestPositionFront2 - ParticlesFront2[p].Position)*(SocialConst*r2);
					ParticlesFront2[p].Velocity = ParticlesFront2[p].Velocity + (personalVelocityFront2 + socialVelocityFront2)*ConstrictionConst;
					ParticlesFront2[p].Velocity.AssuageVelocity();
					ParticlesFront2[p].Position = ParticlesFront2[p].Position + ParticlesFront2[p].Velocity; 
					ParticlesFront2[p].Position.AssuagePosition();
					
					PoseParameters personalVelocityLeft1 = (ParticlesLeft1[p].BestPosition - ParticlesLeft1[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityLeft1 = (GlobalBestPositionLeft1 - ParticlesLeft1[p].Position)*(SocialConst*r2);
					ParticlesLeft1[p].Velocity = ParticlesLeft1[p].Velocity + (personalVelocityLeft1 + socialVelocityLeft1)*ConstrictionConst;
					ParticlesLeft1[p].Velocity.AssuageVelocity();
					ParticlesLeft1[p].Position = ParticlesLeft1[p].Position + ParticlesLeft1[p].Velocity; 
					ParticlesLeft1[p].Position.AssuagePosition();

					PoseParameters personalVelocityLeft2 = (ParticlesLeft2[p].BestPosition - ParticlesLeft2[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityLeft2 = (GlobalBestPositionLeft2 - ParticlesLeft2[p].Position)*(SocialConst*r2);
					ParticlesLeft2[p].Velocity = ParticlesLeft2[p].Velocity + (personalVelocityLeft2 + socialVelocityLeft2)*ConstrictionConst;
					ParticlesLeft2[p].Velocity.AssuageVelocity();
					ParticlesLeft2[p].Position = ParticlesLeft2[p].Position + ParticlesLeft2[p].Velocity; 
					ParticlesLeft2[p].Position.AssuagePosition();

					PoseParameters personalVelocityBack1 = (ParticlesBack1[p].BestPosition - ParticlesBack1[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityBack1 = (GlobalBestPositionBack1 - ParticlesBack1[p].Position)*(SocialConst*r2);
					ParticlesBack1[p].Velocity = ParticlesBack1[p].Velocity + (personalVelocityBack1 + socialVelocityBack1)*ConstrictionConst;
					ParticlesBack1[p].Velocity.AssuageVelocity();
					ParticlesBack1[p].Position = ParticlesBack1[p].Position + ParticlesBack1[p].Velocity; 
					ParticlesBack1[p].Position.AssuagePosition();

					PoseParameters personalVelocityBack2 = (ParticlesBack2[p].BestPosition - ParticlesBack2[p].Position)*(CognitiveConst*r1);
					PoseParameters socialVelocityBack2 = (GlobalBestPositionBack2 - ParticlesBack2[p].Position)*(SocialConst*r2);
					ParticlesBack2[p].Velocity = ParticlesBack2[p].Velocity + (personalVelocityBack2 + socialVelocityBack2)*ConstrictionConst;
					ParticlesBack2[p].Velocity.AssuageVelocity();
					ParticlesBack2[p].Position = ParticlesBack2[p].Position + ParticlesBack2[p].Velocity; 
					ParticlesBack2[p].Position.AssuagePosition();
				}
				
			}	

			std::vector<EnergyPosePair> candidates;
			candidates.push_back(EnergyPosePair(GlobalBestPositionRight1, GlobalBestEnergyRight1));
			candidates.push_back(EnergyPosePair(GlobalBestPositionRight2, GlobalBestEnergyRight2));
			candidates.push_back(EnergyPosePair(GlobalBestPositionFront1, GlobalBestEnergyFront1));
			candidates.push_back(EnergyPosePair(GlobalBestPositionFront2, GlobalBestEnergyFront2));
			candidates.push_back(EnergyPosePair(GlobalBestPositionLeft1, GlobalBestEnergyLeft1));
			candidates.push_back(EnergyPosePair(GlobalBestPositionLeft2, GlobalBestEnergyLeft2));
			candidates.push_back(EnergyPosePair(GlobalBestPositionBack1, GlobalBestEnergyBack1));
			candidates.push_back(EnergyPosePair(GlobalBestPositionBack2, GlobalBestEnergyBack2));
			std::sort(candidates.begin(), candidates.end(), [](const EnergyPosePair& a, const EnergyPosePair& b) -> bool {
				return a.energy < b.energy;
			});

			FiveSet topFive(candidates[0].position, candidates[1].position, candidates[2].position, candidates[3].position, candidates[4].position);

			return topFive; 
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
			glDeleteFramebuffers(1, &picholder);
			glDeleteTextures(1, &pictex);
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
				PoleWriteShader.use();
				PoleWriteShader.setMat4("model", model);
				PoleWriteShader.setMat4("projection", projection);
				PoleWriteShader.setMat4("legrotMatrix", legrotMatrix);
				PoleWriteShader.setMat4("m2bleg", MeshToBoneLeg_L_Pole);
				PoleWriteShader.setMat4("b2mleg", BoneToMeshLeg_L_Pole);
				glBindVertexArray(pole_L.meshes[0].VAO);
				glEnable(GL_DEPTH_TEST);
				glDepthFunc(GL_GREATER);
				glViewport(0, 0, w, h);
				glDrawElements(GL_TRIANGLES, pole_L.meshes[0].indices.size(), GL_UNSIGNED_INT, 0);
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
				PoleWriteShader.use();
				PoleWriteShader.setMat4("model", model);
				PoleWriteShader.setMat4("projection", projection);
				PoleWriteShader.setMat4("legrotMatrix", legrotMatrix);
				PoleWriteShader.setMat4("m2bleg", MeshToBoneLeg_R_Pole);
				PoleWriteShader.setMat4("b2mleg", BoneToMeshLeg_R_Pole);
				glBindVertexArray(pole_R.meshes[0].VAO);
				glEnable(GL_DEPTH_TEST);
				glDepthFunc(GL_GREATER);
				glViewport(0, 0, w, h);
				glDrawElements(GL_TRIANGLES, pole_R.meshes[0].indices.size(), GL_UNSIGNED_INT, 0);
			}

			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
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
		FiveSet FindSolution(float* depth_data, int n, int w, int h, bool is_left, Box& bbox, Intrinsics& intrinsics, int iterations)
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
		
			FiveSet solution = pso.Run(is_left, df_arr, bbox, iterations, projection, -mode, intrinsics);

			return solution;
		}

		void WriteImage(float* currentdt, int n, int w, int h, PoseParameters params, bool is_left, Intrinsics& intrinsics)
		{
			pso.WriteImage(currentdt, w, h, params, is_left, intrinsics);
		}
};
