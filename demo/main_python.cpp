// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2017 NVIDIA Corporation. All rights reserved.

#include "../core/types.h"
#include "../core/maths.h"
#include "../core/platform.h"
#include "../core/mesh.h"
#include "../core/voxelize.h"
#include "../core/sdf.h"
#include "../core/pfm.h"
#include "../core/tga.h"
#include "../core/perlin.h"
#include "../core/convex.h"
#include "../core/cloth.h"

//#include <pybind11.h>
//#include <numpy.h>

#include "../external/SDL2-2.0.4/include/SDL.h"

#include "../include/NvFlex.h"
#include "../include/NvFlexExt.h"
#include "../include/NvFlexDevice.h"

#include <iostream>
#include <map>

#include "shaders.h"
#include "imgui.h"

#include "shadersDemoContext.h"

#if ENABLE_AFTERMATH_SUPPORT
#include <external/GFSDK_Aftermath_v1.21/include/GFSDK_Aftermath.h>
#endif

SDL_Window* g_window;			// window handle
unsigned int g_windowId;		// window id

#define SDL_CONTROLLER_BUTTON_LEFT_TRIGGER (SDL_CONTROLLER_BUTTON_MAX + 1)
#define SDL_CONTROLLER_BUTTON_RIGHT_TRIGGER (SDL_CONTROLLER_BUTTON_MAX + 2)


//
// Gamepad thresholds taken from XINPUT API
//
#define XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE  7849
#define XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE 8689
#define XINPUT_GAMEPAD_TRIGGER_THRESHOLD    30

int deadzones[3] = { XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,
XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE, XINPUT_GAMEPAD_TRIGGER_THRESHOLD };


SDL_GameController* g_gamecontroller = NULL;

using namespace std;

int g_screenWidth = 1280;
int g_screenHeight = 720;
int g_msaaSamples = 8;

int g_numSubsteps;

// a setting of -1 means Flex will use the device specified in the NVIDIA control panel
int g_device = -1;
char g_deviceName[256];

//This should unconstraint the frame rate
bool g_vsync = false;

bool g_benchmark = false;
bool g_extensions = true;
bool g_teamCity = false;
bool g_interop = true;
bool g_d3d12 = false;
bool g_useAsyncCompute = true;
bool g_increaseGfxLoadForAsyncComputeTesting = false;
int g_graphics = 0;	// 0=ogl, 1=DX11, 2=DX12

FluidRenderer* g_fluidRenderer;
FluidRenderBuffers* g_fluidRenderBuffers;
DiffuseRenderBuffers* g_diffuseRenderBuffers;

NvFlexSolver* g_solver;
NvFlexSolverDesc g_solverDesc;
NvFlexLibrary* g_flexLib;
NvFlexParams g_params;
NvFlexTimers g_timers;
int g_numDetailTimers;
NvFlexDetailTimer * g_detailTimers;

int g_maxDiffuseParticles;
int g_maxNeighborsPerParticle;
int g_numExtraParticles;
int g_numExtraMultiplier = 1;
int g_maxContactsPerParticle;

// mesh used for deformable object rendering
Mesh* g_mesh;
vector<int> g_meshSkinIndices;
vector<float> g_meshSkinWeights;
vector<Point3> g_meshRestPositions;
const int g_numSkinWeights = 4;

// mapping of collision mesh to render mesh
std::map<NvFlexConvexMeshId, GpuMesh*> g_convexes;
std::map<NvFlexTriangleMeshId, GpuMesh*> g_meshes;
std::map<NvFlexDistanceFieldId, GpuMesh*> g_fields;

// flag to request collision shapes be updated
bool g_shapesChanged = false;
bool visualize = false;

/* Note that this array of colors is altered by demo code, and is also read from global by graphics API impls */
Colour g_colors[] = { Colour(0.0f, 0.5f, 1.0f), Colour(0.797f, 0.354f, 0.000f),
		Colour(0.092f, 0.465f, 0.820f), Colour(0.000f, 0.349f, 0.173f), Colour(
				0.875f, 0.782f, 0.051f), Colour(0.000f, 0.170f, 0.453f), Colour(
				0.673f, 0.111f, 0.000f), Colour(0.612f, 0.194f, 0.394f) };

struct SimBuffers {
	NvFlexVector<Vec4> positions;
	NvFlexVector<Vec4> restPositions;
	NvFlexVector<Vec3> velocities;
	NvFlexVector<int> phases;
	NvFlexVector<float> densities;
	NvFlexVector<Vec4> anisotropy1;
	NvFlexVector<Vec4> anisotropy2;
	NvFlexVector<Vec4> anisotropy3;
	NvFlexVector<Vec4> normals;
	NvFlexVector<Vec4> smoothPositions;
	NvFlexVector<Vec4> diffusePositions;
	NvFlexVector<Vec4> diffuseVelocities;
	NvFlexVector<int> diffuseCount;

	NvFlexVector<int> activeIndices;

	// convexes
	NvFlexVector<NvFlexCollisionGeometry> shapeGeometry;
	NvFlexVector<Vec4> shapePositions;
	NvFlexVector<Quat> shapeRotations;
	NvFlexVector<Vec4> shapePrevPositions;
	NvFlexVector<Quat> shapePrevRotations;
	NvFlexVector<int> shapeFlags;
	NvFlexVector<Vec3> shapeColors;

	// rigids
	NvFlexVector<int> rigidOffsets;
	NvFlexVector<int> rigidIndices;
	NvFlexVector<int> rigidMeshSize;
	NvFlexVector<float> rigidCoefficients;
	NvFlexVector<float> rigidPlasticThresholds;
	NvFlexVector<float> rigidPlasticCreeps;
	NvFlexVector<Quat> rigidRotations;
	NvFlexVector<Vec3> rigidTranslations;
	NvFlexVector<Vec3> rigidLocalPositions;
	NvFlexVector<Vec4> rigidLocalNormals;

	// inflatables
	NvFlexVector<int> inflatableTriOffsets;
	NvFlexVector<int> inflatableTriCounts;
	NvFlexVector<float> inflatableVolumes;
	NvFlexVector<float> inflatableCoefficients;
	NvFlexVector<float> inflatablePressures;

	// springs
	NvFlexVector<int> springIndices;
	NvFlexVector<float> springLengths;
	NvFlexVector<float> springStiffness;

	NvFlexVector<int> triangles;
	NvFlexVector<Vec3> triangleNormals;
	NvFlexVector<Vec3> uvs;

	SimBuffers(NvFlexLibrary* l) :
			positions(l), restPositions(l), velocities(l), phases(l), densities(
					l), anisotropy1(l), anisotropy2(l), anisotropy3(l), normals(
					l), smoothPositions(l), diffusePositions(l), diffuseVelocities(
					l), diffuseCount(l), activeIndices(l), shapeGeometry(l), shapePositions(
					l), shapeRotations(l), shapePrevPositions(l), shapePrevRotations(
					l), shapeFlags(l),shapeColors(l), rigidOffsets(l), rigidIndices(l), rigidMeshSize(
					l), rigidCoefficients(l), rigidPlasticThresholds(l), rigidPlasticCreeps(
					l), rigidRotations(l), rigidTranslations(l), rigidLocalPositions(
					l), rigidLocalNormals(l), inflatableTriOffsets(l), inflatableTriCounts(
					l), inflatableVolumes(l), inflatableCoefficients(l), inflatablePressures(
					l), springIndices(l), springLengths(l), springStiffness(l), triangles(
					l), triangleNormals(l), uvs(l) {
	}
};

SimBuffers* g_buffers;

void MapBuffers(SimBuffers* buffers) {
	buffers->positions.map();
	buffers->restPositions.map();
	buffers->velocities.map();
	buffers->phases.map();
	buffers->densities.map();
	buffers->anisotropy1.map();
	buffers->anisotropy2.map();
	buffers->anisotropy3.map();
	buffers->normals.map();
	buffers->diffusePositions.map();
	buffers->diffuseVelocities.map();
	buffers->diffuseCount.map();
	buffers->smoothPositions.map();
	buffers->activeIndices.map();

	// convexes
	buffers->shapeGeometry.map();
	buffers->shapePositions.map();
	buffers->shapeRotations.map();
	buffers->shapePrevPositions.map();
	buffers->shapePrevRotations.map();
	buffers->shapeFlags.map();
	buffers->shapeColors.map();

	buffers->rigidOffsets.map();
	buffers->rigidIndices.map();
	buffers->rigidMeshSize.map();
	buffers->rigidCoefficients.map();
	buffers->rigidPlasticThresholds.map();
	buffers->rigidPlasticCreeps.map();
	buffers->rigidRotations.map();
	buffers->rigidTranslations.map();
	buffers->rigidLocalPositions.map();
	buffers->rigidLocalNormals.map();

	buffers->springIndices.map();
	buffers->springLengths.map();
	buffers->springStiffness.map();

	// inflatables
	buffers->inflatableTriOffsets.map();
	buffers->inflatableTriCounts.map();
	buffers->inflatableVolumes.map();
	buffers->inflatableCoefficients.map();
	buffers->inflatablePressures.map();

	buffers->triangles.map();
	buffers->triangleNormals.map();
	buffers->uvs.map();
}

void UnmapBuffers(SimBuffers* buffers) {
	// particles
	buffers->positions.unmap();
	buffers->restPositions.unmap();
	buffers->velocities.unmap();
	buffers->phases.unmap();
	buffers->densities.unmap();
	buffers->anisotropy1.unmap();
	buffers->anisotropy2.unmap();
	buffers->anisotropy3.unmap();
	buffers->normals.unmap();
	buffers->diffusePositions.unmap();
	buffers->diffuseVelocities.unmap();
	buffers->diffuseCount.unmap();
	buffers->smoothPositions.unmap();
	buffers->activeIndices.unmap();

	// convexes
	buffers->shapeGeometry.unmap();
	buffers->shapePositions.unmap();
	buffers->shapeRotations.unmap();
	buffers->shapePrevPositions.unmap();
	buffers->shapePrevRotations.unmap();
	buffers->shapeFlags.unmap();

	buffers->shapeColors.unmap();

	// rigids
	buffers->rigidOffsets.unmap();
	buffers->rigidIndices.unmap();
	buffers->rigidMeshSize.unmap();
	buffers->rigidCoefficients.unmap();
	buffers->rigidPlasticThresholds.unmap();
	buffers->rigidPlasticCreeps.unmap();
	buffers->rigidRotations.unmap();
	buffers->rigidTranslations.unmap();
	buffers->rigidLocalPositions.unmap();
	buffers->rigidLocalNormals.unmap();

	// springs
	buffers->springIndices.unmap();
	buffers->springLengths.unmap();
	buffers->springStiffness.unmap();

	// inflatables
	buffers->inflatableTriOffsets.unmap();
	buffers->inflatableTriCounts.unmap();
	buffers->inflatableVolumes.unmap();
	buffers->inflatableCoefficients.unmap();
	buffers->inflatablePressures.unmap();

	// triangles
	buffers->triangles.unmap();
	buffers->triangleNormals.unmap();
	buffers->uvs.unmap();

}

SimBuffers* AllocBuffers(NvFlexLibrary* lib) {
	return new SimBuffers(lib);
}

void DestroyBuffers(SimBuffers* buffers) {
	// particles
	buffers->positions.destroy();
	buffers->restPositions.destroy();
	buffers->velocities.destroy();
	buffers->phases.destroy();
	buffers->densities.destroy();
	buffers->anisotropy1.destroy();
	buffers->anisotropy2.destroy();
	buffers->anisotropy3.destroy();
	buffers->normals.destroy();
	buffers->diffusePositions.destroy();
	buffers->diffuseVelocities.destroy();
	buffers->diffuseCount.destroy();
	buffers->smoothPositions.destroy();
	buffers->activeIndices.destroy();

	// convexes
	buffers->shapeGeometry.destroy();
	buffers->shapePositions.destroy();
	buffers->shapeRotations.destroy();
	buffers->shapePrevPositions.destroy();
	buffers->shapePrevRotations.destroy();
	buffers->shapeFlags.destroy();

	buffers->shapeColors.destroy();

	// rigids
	buffers->rigidOffsets.destroy();
	buffers->rigidIndices.destroy();
	buffers->rigidMeshSize.destroy();
	buffers->rigidCoefficients.destroy();
	buffers->rigidPlasticThresholds.destroy();
	buffers->rigidPlasticCreeps.destroy();
	buffers->rigidRotations.destroy();
	buffers->rigidTranslations.destroy();
	buffers->rigidLocalPositions.destroy();
	buffers->rigidLocalNormals.destroy();

	// springs
	buffers->springIndices.destroy();
	buffers->springLengths.destroy();
	buffers->springStiffness.destroy();

	// inflatables
	buffers->inflatableTriOffsets.destroy();
	buffers->inflatableTriCounts.destroy();
	buffers->inflatableVolumes.destroy();
	buffers->inflatableCoefficients.destroy();
	buffers->inflatablePressures.destroy();

	// triangles
	buffers->triangles.destroy();
	buffers->triangleNormals.destroy();
	buffers->uvs.destroy();

	delete buffers;
}

Vec3 g_camPos(-1.68645, 30.2395, 13.6924);
Vec3 g_camAngle(0.340339, -0.869178, 0);

Vec3 g_camVel(0.0f);
Vec3 g_camSmoothVel(0.0f);

float g_camSpeed;
float g_camNear;
float g_camFar;

Vec3 g_lightPos;
Vec3 g_lightDir;
Vec3 g_lightTarget;

bool g_pause = false;
bool g_step = false;
bool g_capture = false;
bool g_showHelp = false;
bool g_tweakPanel = true;
bool g_fullscreen = false;
bool g_wireframe = false;
bool g_debug = false;

bool g_emit = false;
bool g_warmup = false;

float g_windTime = 0.0f;
float g_windFrequency = 0.1f;
float g_windStrength = 0.0f;

bool g_wavePool = false;
float g_waveTime = 0.0f;
float g_wavePlane;
float g_waveFrequency = 1.5f;
float g_waveAmplitude = 1.0f;
float g_waveFloorTilt = 0.0f;

Vec3 g_sceneLower;
Vec3 g_sceneUpper;

float g_blur;
float g_ior;
bool g_drawEllipsoids;
bool g_drawPoints;
bool g_drawMesh;
bool g_drawCloth;
float g_expandCloth;// amount to expand cloth along normal (to account for particle radius)

bool g_drawOpaque;
int g_drawSprings;		// 0: no draw, 1: draw stretch 2: draw tether
bool g_drawBases = false;
bool g_drawContacts = false;
bool g_drawNormals = false;
bool g_drawDiffuse;
bool g_drawShapeGrid = false;
bool g_drawDensity = false;
bool g_drawRopes;
float g_pointScale;
float g_ropeScale;
float g_drawPlaneBias;	// move planes along their normal for rendering

float g_diffuseScale;
float g_diffuseMotionScale;
bool g_diffuseShadow;
float g_diffuseInscatter;
float g_diffuseOutscatter;

float g_dt = 1.0f / 60.0f;	// the time delta used for simulation
float g_realdt;				// the real world time delta between updates

float g_waitTime;		// the CPU time spent waiting for the GPU
float g_updateTime;     // the CPU time spent on Flex
float g_renderTime;		// the CPU time spent calling OpenGL to render the scene
// the above times don't include waiting for vsync
float g_simLatency; // the time the GPU spent between the first and last NvFlexUpdateSolver() operation. Because some GPUs context switch, this can include graphics time.

int g_scene = 0;
int g_selectedScene = g_scene;
int g_levelScroll;			// offset for level selection scroll area
bool g_resetScene = false; //if the user clicks the reset button or presses the reset key this is set to true;

int g_frame = 0;
int g_numSolidParticles = 0;

int g_mouseParticle = -1;
float g_mouseT = 0.0f;
Vec3 g_mousePos;
float g_mouseMass;
bool g_mousePicked = false;

float z_pos_array[3548];
float y_pos_array[3548];
float x_pos_array[3548];

Eigen::MatrixXd curr_state;

// mouse
int g_lastx;
int g_lasty;
int g_lastb = -1;

bool g_profile = false;
bool g_outputAllFrameTimes = false;
bool g_asyncComputeBenchmark = false;

ShadowMap* g_shadowMap;

Vec4 g_fluidColor;
Vec4 g_diffuseColor;
Vec3 g_meshColor;
Vec3 g_clearColor;
float g_lightDistance;
float g_fogDistance;

FILE* g_ffmpeg;

void DrawShapes();

class Scene;
vector<Scene*> g_scenes;

struct Emitter {
	Emitter() :
			mSpeed(0.0f), mEnabled(false), mLeftOver(0.0f), mWidth(8) {
	}

	Vec3 mPos;
	Vec3 mDir;
	Vec3 mRight;
	float mSpeed;
	bool mEnabled;
	float mLeftOver;
	int mWidth;
};

vector<Emitter> g_emitters(1);	// first emitter is the camera 'gun'

struct Rope {
	std::vector<int> mIndices;
};

vector<Rope> g_ropes;

inline float sqr(float x) {
	return x * x;
}

#include "helpers.h"
#include "scenes.h"
#include "benchmark.h"

SDL_Event e;

Eigen::MatrixXd UpdateControlScene(Eigen::VectorXd act) {
	// give scene a chance to make changes to particle buffers
	return g_scenes[g_scene]->Update(act);
}

void Init(bool centerCamera = true) {

	RandInit();

	if (g_solver) {
		if (g_buffers)
			DestroyBuffers(g_buffers);

		if (visualize) {
			DestroyFluidRenderBuffers(g_fluidRenderBuffers);
			DestroyDiffuseRenderBuffers(g_diffuseRenderBuffers);
		}

		for (auto& iter : g_meshes) {
			NvFlexDestroyTriangleMesh(g_flexLib, iter.first);
			DestroyGpuMesh(iter.second);
		}

		for (auto& iter : g_fields) {
			NvFlexDestroyDistanceField(g_flexLib, iter.first);
			DestroyGpuMesh(iter.second);
		}

		for (auto& iter : g_convexes) {
			NvFlexDestroyConvexMesh(g_flexLib, iter.first);
			DestroyGpuMesh(iter.second);
		}

		g_fields.clear();
		g_meshes.clear();
		g_convexes.clear();

		NvFlexDestroySolver(g_solver);
		g_solver = NULL;
	}

	// alloc buffers
	g_buffers = AllocBuffers(g_flexLib);

	// map during initialization
	MapBuffers(g_buffers);

	g_buffers->positions.resize(0);
	g_buffers->velocities.resize(0);
	g_buffers->phases.resize(0);

	g_buffers->rigidOffsets.resize(0);
	g_buffers->rigidIndices.resize(0);
	g_buffers->rigidMeshSize.resize(0);
	g_buffers->rigidRotations.resize(0);
	g_buffers->rigidTranslations.resize(0);
	g_buffers->rigidCoefficients.resize(0);
	g_buffers->rigidPlasticThresholds.resize(0);
	g_buffers->rigidPlasticCreeps.resize(0);
	g_buffers->rigidLocalPositions.resize(0);
	g_buffers->rigidLocalNormals.resize(0);

	g_buffers->springIndices.resize(0);
	g_buffers->springLengths.resize(0);
	g_buffers->springStiffness.resize(0);
	g_buffers->triangles.resize(0);
	g_buffers->triangleNormals.resize(0);
	g_buffers->uvs.resize(0);

	g_meshSkinIndices.resize(0);
	g_meshSkinWeights.resize(0);

	g_emitters.resize(1);
	g_emitters[0].mEnabled = false;
	g_emitters[0].mSpeed = 1.0f;
	g_emitters[0].mLeftOver = 0.0f;
	g_emitters[0].mWidth = 8;

	g_buffers->shapeGeometry.resize(0);
	g_buffers->shapePositions.resize(0);
	g_buffers->shapeRotations.resize(0);
	g_buffers->shapePrevPositions.resize(0);
	g_buffers->shapePrevRotations.resize(0);
	g_buffers->shapeFlags.resize(0);

	g_ropes.resize(0);

	// remove collision shapes
	delete g_mesh;
	g_mesh = NULL;

	g_frame = 0;
	g_pause = false;

//	g_dt = 1.0f / 60.0f;
	g_waveTime = 0.0f;
	g_windTime = 0.0f;
	g_windStrength = 1.0f;

	g_blur = 1.0f;
	g_fluidColor = Vec4(0.1f, 0.4f, 0.8f, 1.0f);
	g_meshColor = Vec3(0.9f, 0.9f, 0.9f);
	g_drawEllipsoids = false;
	g_drawPoints = true;
	g_drawCloth = true;
	g_expandCloth = 0.0f;

	g_drawOpaque = false;
	g_drawSprings = false;
	g_drawDiffuse = false;
	g_drawMesh = true;
	g_drawRopes = true;
	g_drawDensity = false;
	g_ior = 1.0f;
	g_lightDistance = 2.0f;
	g_fogDistance = 0.005f;

	g_camSpeed = 0.075f;
	g_camNear = 0.01f;
	g_camFar = 1000.0f;

	g_pointScale = 1.0f;
	g_ropeScale = 1.0f;
	g_drawPlaneBias = 0.0f;

	// sim params
	g_params.gravity[0] = 0.0f;
	g_params.gravity[1] = -9.8f;
	g_params.gravity[2] = 0.0f;

	g_params.wind[0] = 0.0f;
	g_params.wind[1] = 0.0f;
	g_params.wind[2] = 0.0f;

	g_params.radius = 0.15f;
	g_params.viscosity = 0.0f;
	g_params.dynamicFriction = 0.0f;
	g_params.staticFriction = 0.0f;
	g_params.particleFriction = 0.0f; // scale friction between particles by default
	g_params.freeSurfaceDrag = 0.0f;
	g_params.drag = 0.0f;
	g_params.lift = 0.0f;
	g_params.numIterations = 3;
	g_params.fluidRestDistance = 0.0f;
	g_params.solidRestDistance = 0.0f;

	g_params.anisotropyScale = 1.0f;
	g_params.anisotropyMin = 0.1f;
	g_params.anisotropyMax = 2.0f;
	g_params.smoothing = 1.0f;

	g_params.dissipation = 0.0f;
	g_params.damping = 0.0f;
	g_params.particleCollisionMargin = 0.0f;
	g_params.shapeCollisionMargin = 0.0f;
	g_params.collisionDistance = 0.0f;
	g_params.sleepThreshold = 0.0f;
	g_params.shockPropagation = 0.0f;
	g_params.restitution = 0.0f;

	g_params.maxSpeed = FLT_MAX;
	g_params.maxAcceleration = 100.0f;	// approximately 10x gravity

	g_params.relaxationMode = eNvFlexRelaxationLocal;
	g_params.relaxationFactor = 1.0f;
	g_params.solidPressure = 1.0f;
	g_params.adhesion = 0.0f;
	g_params.cohesion = 0.025f;
	g_params.surfaceTension = 0.0f;
	g_params.vorticityConfinement = 0.0f;
	g_params.buoyancy = 1.0f;
	g_params.diffuseThreshold = 100.0f;
	g_params.diffuseBuoyancy = 1.0f;
	g_params.diffuseDrag = 0.8f;
	g_params.diffuseBallistic = 16;
	g_params.diffuseLifetime = 2.0f;

	g_numSubsteps = 2;

	// planes created after particles
	g_params.numPlanes = 1;

	g_diffuseScale = 0.5f;
	g_diffuseColor = 1.0f;
	g_diffuseMotionScale = 1.0f;
	g_diffuseShadow = false;
	g_diffuseInscatter = 0.8f;
	g_diffuseOutscatter = 0.53f;

	// reset phase 0 particle color to blue
	g_colors[0] = Colour(0.0f, 0.5f, 1.0f);

	g_numSolidParticles = 0;

	g_waveFrequency = 1.5f;
	g_waveAmplitude = 1.5f;
	g_waveFloorTilt = 0.0f;
	g_emit = false;
	g_warmup = false;

	g_mouseParticle = -1;

	g_maxDiffuseParticles = 0;	// number of diffuse particles
	g_maxNeighborsPerParticle = 96;
	g_numExtraParticles = 0;// number of particles allocated but not made active
	g_maxContactsPerParticle = 6;

	g_sceneLower = FLT_MAX;
	g_sceneUpper = -FLT_MAX;

	// initialize solver desc
	NvFlexSetSolverDescDefaults(&g_solverDesc);

	// create scene
	StartGpuWork();
	curr_state = g_scenes[g_scene]->Initialize(0);
	EndGpuWork();

	uint32_t numParticles = g_buffers->positions.size();
	uint32_t maxParticles = numParticles
			+ g_numExtraParticles * g_numExtraMultiplier;

	if (g_params.solidRestDistance == 0.0f)
		g_params.solidRestDistance = g_params.radius;

	// if fluid present then we assume solid particles have the same radius
	if (g_params.fluidRestDistance > 0.0f)
		g_params.solidRestDistance = g_params.fluidRestDistance;

	// set collision distance automatically based on rest distance if not alraedy set
	if (g_params.collisionDistance == 0.0f)
		g_params.collisionDistance = Max(g_params.solidRestDistance,
				g_params.fluidRestDistance) * 0.5f;

	// default particle friction to 10% of shape friction
	if (g_params.particleFriction == 0.0f)
		g_params.particleFriction = g_params.dynamicFriction * 0.1f;

	// add a margin for detecting contacts between particles and shapes
	if (g_params.shapeCollisionMargin == 0.0f)
		g_params.shapeCollisionMargin = g_params.collisionDistance * 0.5f;

	// calculate particle bounds
	Vec3 particleLower, particleUpper;
	GetParticleBounds(particleLower, particleUpper);

	// accommodate shapes
	Vec3 shapeLower, shapeUpper;
	GetShapeBounds(shapeLower, shapeUpper);

	// update bounds
	g_sceneLower = Min(Min(g_sceneLower, particleLower), shapeLower);
	g_sceneUpper = Max(Max(g_sceneUpper, particleUpper), shapeUpper);

	g_sceneLower -= g_params.collisionDistance;
	g_sceneUpper += g_params.collisionDistance;

	// update collision planes to match flexs
	Vec3 up = Normalize(Vec3(-g_waveFloorTilt, 1.0f, 0.0f));

	(Vec4&) g_params.planes[0] = Vec4(up.x, up.y, up.z, 0.0f);
	(Vec4&) g_params.planes[1] = Vec4(0.0f, 0.0f, 1.0f, -g_sceneLower.z);
	(Vec4&) g_params.planes[2] = Vec4(1.0f, 0.0f, 0.0f, -g_sceneLower.x);
	(Vec4&) g_params.planes[3] = Vec4(-1.0f, 0.0f, 0.0f, g_sceneUpper.x);
	(Vec4&) g_params.planes[4] = Vec4(0.0f, 0.0f, -1.0f, g_sceneUpper.z);
	(Vec4&) g_params.planes[5] = Vec4(0.0f, -1.0f, 0.0f, g_sceneUpper.y);

	g_wavePlane = g_params.planes[2][3];

	g_buffers->diffusePositions.resize(g_maxDiffuseParticles);
	g_buffers->diffuseVelocities.resize(g_maxDiffuseParticles);
	g_buffers->diffuseCount.resize(1, 0);

	// for fluid rendering these are the Laplacian smoothed positions
	g_buffers->smoothPositions.resize(maxParticles);

	g_buffers->normals.resize(0);
	g_buffers->normals.resize(maxParticles);

	// initialize normals (just for rendering before simulation starts)
	int numTris = g_buffers->triangles.size() / 3;
	for (int i = 0; i < numTris; ++i) {
		Vec3 v0 = Vec3(g_buffers->positions[g_buffers->triangles[i * 3 + 0]]);
		Vec3 v1 = Vec3(g_buffers->positions[g_buffers->triangles[i * 3 + 1]]);
		Vec3 v2 = Vec3(g_buffers->positions[g_buffers->triangles[i * 3 + 2]]);

		Vec3 n = Cross(v1 - v0, v2 - v0);

		g_buffers->normals[g_buffers->triangles[i * 3 + 0]] += Vec4(n, 0.0f);
		g_buffers->normals[g_buffers->triangles[i * 3 + 1]] += Vec4(n, 0.0f);
		g_buffers->normals[g_buffers->triangles[i * 3 + 2]] += Vec4(n, 0.0f);
	}

	for (int i = 0; i < int(maxParticles); ++i)
		g_buffers->normals[i] = Vec4(
				SafeNormalize(Vec3(g_buffers->normals[i]),
						Vec3(0.0f, 1.0f, 0.0f)), 0.0f);

	// save mesh positions for skinning
	if (g_mesh) {
		g_meshRestPositions = g_mesh->m_positions;
	} else {
		g_meshRestPositions.resize(0);
	}

	g_solverDesc.maxParticles = maxParticles;
	g_solverDesc.maxDiffuseParticles = g_maxDiffuseParticles;
	g_solverDesc.maxNeighborsPerParticle = g_maxNeighborsPerParticle;
	g_solverDesc.maxContactsPerParticle = g_maxContactsPerParticle;

	// main create method for the Flex solver
	g_solver = NvFlexCreateSolver(g_flexLib, &g_solverDesc);

	// give scene a chance to do some post solver initialization
	g_scenes[g_scene]->PostInitialize();

	// center camera on particles
	if (centerCamera) {
		g_camPos = Vec3((g_sceneLower.x + g_sceneUpper.x) * 0.5f,
				min(g_sceneUpper.y * 1.25f, 6.0f),
				g_sceneUpper.z + min(g_sceneUpper.y, 6.0f) * 2.0f);
		g_camAngle = Vec3(0.0f, -DegToRad(15.0f), 0.0f);

		// give scene a chance to modify camera position
		g_scenes[g_scene]->CenterCamera();
	}

	// create active indices (just a contiguous block for the demo)
	g_buffers->activeIndices.resize(g_buffers->positions.size());
	for (int i = 0; i < g_buffers->activeIndices.size(); ++i)
		g_buffers->activeIndices[i] = i;

	// resize particle buffers to fit
	g_buffers->positions.resize(maxParticles);
	g_buffers->velocities.resize(maxParticles);
	g_buffers->phases.resize(maxParticles);

	g_buffers->densities.resize(maxParticles);
	g_buffers->anisotropy1.resize(maxParticles);
	g_buffers->anisotropy2.resize(maxParticles);
	g_buffers->anisotropy3.resize(maxParticles);

	// save rest positions
	g_buffers->restPositions.resize(g_buffers->positions.size());
	for (int i = 0; i < g_buffers->positions.size(); ++i)
		g_buffers->restPositions[i] = g_buffers->positions[i];

	// builds rigids constraints
	if (g_buffers->rigidOffsets.size()) {
		assert(g_buffers->rigidOffsets.size() > 1);

		const int numRigids = g_buffers->rigidOffsets.size() - 1;

		// If the centers of mass for the rigids are not yet computed, this is done here
		// (If the CreateParticleShape method is used instead of the NvFlexExt methods, the centers of mass will be calculated here)
		if (g_buffers->rigidTranslations.size() == 0) {
			g_buffers->rigidTranslations.resize(
					g_buffers->rigidOffsets.size() - 1, Vec3());
			CalculateRigidCentersOfMass(&g_buffers->positions[0],
					g_buffers->positions.size(), &g_buffers->rigidOffsets[0],
					&g_buffers->rigidTranslations[0],
					&g_buffers->rigidIndices[0], numRigids);
		}

		// calculate local rest space positions
		g_buffers->rigidLocalPositions.resize(g_buffers->rigidOffsets.back());
		CalculateRigidLocalPositions(&g_buffers->positions[0],
				&g_buffers->rigidOffsets[0], &g_buffers->rigidTranslations[0],
				&g_buffers->rigidIndices[0], numRigids,
				&g_buffers->rigidLocalPositions[0]);

		// set rigidRotations to correct length, probably NULL up until here
		g_buffers->rigidRotations.resize(g_buffers->rigidOffsets.size() - 1,
				Quat());
	}

	// unmap so we can start transferring data to GPU
	UnmapBuffers(g_buffers);

	//-----------------------------
	// Send data to Flex

	NvFlexCopyDesc copyDesc;
	copyDesc.dstOffset = 0;
	copyDesc.srcOffset = 0;
	copyDesc.elementCount = numParticles;

	NvFlexSetParams(g_solver, &g_params);
	NvFlexSetParticles(g_solver, g_buffers->positions.buffer, &copyDesc);
	NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, &copyDesc);
	NvFlexSetNormals(g_solver, g_buffers->normals.buffer, &copyDesc);
	NvFlexSetPhases(g_solver, g_buffers->phases.buffer, &copyDesc);
	NvFlexSetRestParticles(g_solver, g_buffers->restPositions.buffer,
			&copyDesc);

	NvFlexSetActive(g_solver, g_buffers->activeIndices.buffer, &copyDesc);
	NvFlexSetActiveCount(g_solver, numParticles);

	// springs
	if (g_buffers->springIndices.size()) {
		assert((g_buffers->springIndices.size() & 1) == 0);
		assert(
				(g_buffers->springIndices.size() / 2)
						== g_buffers->springLengths.size());

		NvFlexSetSprings(g_solver, g_buffers->springIndices.buffer,
				g_buffers->springLengths.buffer,
				g_buffers->springStiffness.buffer,
				g_buffers->springLengths.size());
	}

	// rigids
	if (g_buffers->rigidOffsets.size()) {
		NvFlexSetRigids(g_solver, g_buffers->rigidOffsets.buffer,
				g_buffers->rigidIndices.buffer,
				g_buffers->rigidLocalPositions.buffer,
				g_buffers->rigidLocalNormals.buffer,
				g_buffers->rigidCoefficients.buffer,
				g_buffers->rigidPlasticThresholds.buffer,
				g_buffers->rigidPlasticCreeps.buffer,
				g_buffers->rigidRotations.buffer,
				g_buffers->rigidTranslations.buffer,
				g_buffers->rigidOffsets.size() - 1,
				g_buffers->rigidIndices.size());
	}

	// inflatables
	if (g_buffers->inflatableTriOffsets.size()) {
		NvFlexSetInflatables(g_solver, g_buffers->inflatableTriOffsets.buffer,
				g_buffers->inflatableTriCounts.buffer,
				g_buffers->inflatableVolumes.buffer,
				g_buffers->inflatablePressures.buffer,
				g_buffers->inflatableCoefficients.buffer,
				g_buffers->inflatableTriOffsets.size());
	}

	// dynamic triangles
	if (g_buffers->triangles.size()) {
		NvFlexSetDynamicTriangles(g_solver, g_buffers->triangles.buffer,
				g_buffers->triangleNormals.buffer,
				g_buffers->triangles.size() / 3);
	}

	// collision shapes
	if (g_buffers->shapeFlags.size()) {
		NvFlexSetShapes(g_solver, g_buffers->shapeGeometry.buffer,
				g_buffers->shapePositions.buffer,
				g_buffers->shapeRotations.buffer,
				g_buffers->shapePrevPositions.buffer,
				g_buffers->shapePrevRotations.buffer,
				g_buffers->shapeFlags.buffer,
				int(g_buffers->shapeFlags.size()));
	}
	if (visualize) {
		// create render buffers
		g_fluidRenderBuffers = CreateFluidRenderBuffers(maxParticles,
				g_interop);
		g_diffuseRenderBuffers = CreateDiffuseRenderBuffers(
				g_maxDiffuseParticles, g_interop);
	}
	// perform initial sim warm up
	if (g_warmup) {
		printf("Warming up sim..\n");

		// warm it up (relax positions to reach rest density without affecting velocity)
		NvFlexParams copy = g_params;
		copy.numIterations = 4;

		NvFlexSetParams(g_solver, &copy);

		const int kWarmupIterations = 100;

		for (int i = 0; i < kWarmupIterations; ++i) {
			NvFlexUpdateSolver(g_solver, 0.0001f, 1, false);
			NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
		}

		// udpate host copy
		NvFlexGetParticles(g_solver, g_buffers->positions.buffer, NULL);
		NvFlexGetSmoothParticles(g_solver, g_buffers->smoothPositions.buffer,
		NULL);
		NvFlexGetAnisotropy(g_solver, g_buffers->anisotropy1.buffer,
				g_buffers->anisotropy2.buffer, g_buffers->anisotropy3.buffer,
				NULL);

		printf("Finished warm up.\n");
	}

}
Eigen::MatrixXd Reset() {
	Init(false);
	return curr_state;
}

void Shutdown() {
	// free buffers
	DestroyBuffers(g_buffers);

	for (auto& iter : g_meshes) {
		NvFlexDestroyTriangleMesh(g_flexLib, iter.first);
		DestroyGpuMesh(iter.second);
	}

	for (auto& iter : g_fields) {
		NvFlexDestroyDistanceField(g_flexLib, iter.first);
		DestroyGpuMesh(iter.second);
	}

	for (auto& iter : g_convexes) {
		NvFlexDestroyConvexMesh(g_flexLib, iter.first);
		DestroyGpuMesh(iter.second);
	}

	g_fields.clear();
	g_meshes.clear();

	NvFlexDestroySolver(g_solver);
	NvFlexShutdown(g_flexLib);

}

void UpdateCamera() {
	Vec3 forward(-sinf(g_camAngle.x) * cosf(g_camAngle.y), sinf(g_camAngle.y),
			-cosf(g_camAngle.x) * cosf(g_camAngle.y));
	Vec3 right(Normalize(Cross(forward, Vec3(0.0f, 1.0f, 0.0f))));

	g_camSmoothVel = Lerp(g_camSmoothVel, g_camVel, 0.1f);
	g_camPos += (forward * g_camSmoothVel.z + right * g_camSmoothVel.x
			+ Cross(right, forward) * g_camSmoothVel.y);
}

void UpdateMouse() {
	// mouse button is up release particle
	if (g_lastb == -1) {
		if (g_mouseParticle != -1) {
			// restore particle mass
			g_buffers->positions[g_mouseParticle].w = g_mouseMass;

			// deselect
			g_mouseParticle = -1;
		}
	}

	// mouse went down, pick new particle
	if (g_mousePicked) {
		assert(g_mouseParticle == -1);

		Vec3 origin, dir;
		GetViewRay(g_lastx, g_screenHeight - g_lasty, origin, dir);

		const int numActive = NvFlexGetActiveCount(g_solver);

		g_mouseParticle = PickParticle(origin, dir, &g_buffers->positions[0],
				&g_buffers->phases[0], numActive, g_params.radius * 0.8f,
				g_mouseT);

		if (g_mouseParticle != -1) {
			printf("picked: %d, p: %f %f %f\n", g_mouseParticle,
					g_buffers->positions[g_mouseParticle].x,
					g_buffers->positions[g_mouseParticle].y,
					g_buffers->positions[g_mouseParticle].z);
			//printf("picked: %d, mass: %f v: %f %f %f\n", g_mouseParticle, g_buffers->positions[g_mouseParticle].w, g_buffers->velocities[g_mouseParticle].x, g_buffers->velocities[g_mouseParticle].y, g_buffers->velocities[g_mouseParticle].z);

			g_mousePos = origin + dir * g_mouseT;
			g_mouseMass = g_buffers->positions[g_mouseParticle].w;
			g_buffers->positions[g_mouseParticle].w = 0.0f;	// increase picked particle's mass to force it towards the point
		}

		g_mousePicked = false;
	}

	// update picked particle position
	if (g_mouseParticle != -1) {
		Vec3 p = Lerp(Vec3(g_buffers->positions[g_mouseParticle]), g_mousePos,
				0.8f);
		Vec3 delta = p - Vec3(g_buffers->positions[g_mouseParticle]);

		g_buffers->positions[g_mouseParticle].x = p.x;
		g_buffers->positions[g_mouseParticle].y = p.y;
		g_buffers->positions[g_mouseParticle].z = p.z;

		g_buffers->velocities[g_mouseParticle].x = delta.x / g_dt;
		g_buffers->velocities[g_mouseParticle].y = delta.y / g_dt;
		g_buffers->velocities[g_mouseParticle].z = delta.z / g_dt;
	}
}

void UpdateWind() {
	g_windTime += g_dt;

	const Vec3 kWindDir = Vec3(3.0f, 15.0f, 0.0f);
	const float kNoise = Perlin1D(g_windTime * g_windFrequency, 10, 0.25f);
	Vec3 wind = g_windStrength * kWindDir * Vec3(kNoise, fabsf(kNoise), 0.0f);

	g_params.wind[0] = wind.x;
	g_params.wind[1] = wind.y;
	g_params.wind[2] = wind.z;

	if (g_wavePool) {
		g_waveTime += g_dt;

		g_params.planes[2][3] = g_wavePlane
				+ (sinf(float(g_waveTime) * g_waveFrequency - kPi * 0.5f) * 0.5f
						+ 0.5f) * g_waveAmplitude;
	}
}

void SyncScene() {
	// let the scene send updates to flex directly
	g_scenes[g_scene]->Sync();
}

void UpdateScene() {
	// give scene a chance to make changes to particle buffers
	g_scenes[g_scene]->Update();
}

void RenderScene() {
	const int numParticles = NvFlexGetActiveCount(g_solver);
	const int numDiffuse = g_buffers->diffuseCount[0];

	//---------------------------------------------------
	// use VBO buffer wrappers to allow Flex to write directly to the OpenGL buffers
	// Flex will take care of any CUDA interop mapping/unmapping during the get() operations

	if (numParticles) {

		if (g_interop) {
			// copy data directly from solver to the renderer buffers
			UpdateFluidRenderBuffers(g_fluidRenderBuffers, g_solver,
					g_drawEllipsoids, g_drawDensity);
		} else {
			// copy particle data to GPU render device

			if (g_drawEllipsoids) {
				// if fluid surface rendering then update with smooth positions and anisotropy
				UpdateFluidRenderBuffers(g_fluidRenderBuffers,
						&g_buffers->smoothPositions[0],
						(g_drawDensity) ?
								&g_buffers->densities[0] :
								(float*) &g_buffers->phases[0],
						&g_buffers->anisotropy1[0], &g_buffers->anisotropy2[0],
						&g_buffers->anisotropy3[0], g_buffers->positions.size(),
						&g_buffers->activeIndices[0], numParticles);
			} else {
				// otherwise just send regular positions and no anisotropy
				UpdateFluidRenderBuffers(g_fluidRenderBuffers,
						&g_buffers->positions[0],
						(float*) &g_buffers->phases[0],
						NULL, NULL, NULL, g_buffers->positions.size(),
						&g_buffers->activeIndices[0], numParticles);
			}
		}
	}

	// GPU Render time doesn't include CPU->GPU copy time
	GraphicsTimerBegin();

	if (numDiffuse) {
		if (g_interop) {
			// copy data directly from solver to the renderer buffers
			UpdateDiffuseRenderBuffers(g_diffuseRenderBuffers, g_solver);
		} else {
			// copy diffuse particle data from host to GPU render device
			UpdateDiffuseRenderBuffers(g_diffuseRenderBuffers,
					&g_buffers->diffusePositions[0],
					&g_buffers->diffuseVelocities[0], numDiffuse);
		}
	}

	//---------------------------------------
	// setup view and state

	float fov = kPi / 4.0f;
	float aspect = float(g_screenWidth) / g_screenHeight;

	Matrix44 proj = ProjectionMatrix(RadToDeg(fov), aspect, g_camNear,
			g_camFar);
	Matrix44 view = RotationMatrix(-g_camAngle.x, Vec3(0.0f, 1.0f, 0.0f))
			* RotationMatrix(-g_camAngle.y,
					Vec3(cosf(-g_camAngle.x), 0.0f, sinf(-g_camAngle.x)))
			* TranslationMatrix(-Point3(g_camPos));

	//------------------------------------
	// lighting pass

	// expand scene bounds to fit most scenes
	g_sceneLower = Min(g_sceneLower, Vec3(-2.0f, 0.0f, -2.0f));
	g_sceneUpper = Max(g_sceneUpper, Vec3(2.0f, 2.0f, 2.0f));

	Vec3 sceneExtents = g_sceneUpper - g_sceneLower;
	Vec3 sceneCenter = 0.5f * (g_sceneUpper + g_sceneLower);

	g_lightDir = Normalize(Vec3(5.0f, 15.0f, 7.5f));
	g_lightPos = sceneCenter
			+ g_lightDir * Length(sceneExtents) * g_lightDistance;
	g_lightTarget = sceneCenter;

	// calculate tight bounds for shadow frustum
	float lightFov = 2.0f
			* atanf(
					Length(g_sceneUpper - sceneCenter)
							/ Length(g_lightPos - sceneCenter));

	// scale and clamp fov for aesthetics
	lightFov = Clamp(lightFov, DegToRad(25.0f), DegToRad(65.0f));

	Matrix44 lightPerspective = ProjectionMatrix(RadToDeg(lightFov), 1.0f, 1.0f,
			1000.0f);
	Matrix44 lightView = LookAtMatrix(Point3(g_lightPos),
			Point3(g_lightTarget));
	Matrix44 lightTransform = lightPerspective * lightView;

	// radius used for drawing
	float radius = Max(g_params.solidRestDistance, g_params.fluidRestDistance)
			* 0.5f * g_pointScale;

	//-------------------------------------
	// shadowing pass

	if (g_meshSkinIndices.size())
		SkinMesh();

	// create shadow maps
	ShadowBegin(g_shadowMap);

	SetView(lightView, lightPerspective);
	SetCullMode(false);

	// give scene a chance to do custom drawing
	g_scenes[g_scene]->Draw(1);

	if (g_drawMesh)
		DrawMesh(g_mesh, g_meshColor);

	DrawShapes();

	if (g_drawCloth && g_buffers->triangles.size()) {
		DrawCloth(&g_buffers->positions[0], &g_buffers->normals[0],
				g_buffers->uvs.size() ? &g_buffers->uvs[0].x : NULL,
				&g_buffers->triangles[0], g_buffers->triangles.size() / 3,
				g_buffers->positions.size(), 3, g_expandCloth);
	}

	if (g_drawRopes) {
		for (size_t i = 0; i < g_ropes.size(); ++i)
			DrawRope(&g_buffers->positions[0], &g_ropes[i].mIndices[0],
					g_ropes[i].mIndices.size(), radius * g_ropeScale, i);
	}

	int shadowParticles = numParticles;
	int shadowParticlesOffset = 0;

	if (!g_drawPoints) {
		shadowParticles = 0;

		if (g_drawEllipsoids) {
			shadowParticles = numParticles - g_numSolidParticles;
			shadowParticlesOffset = g_numSolidParticles;
		}
	} else {
		int offset = g_drawMesh ? g_numSolidParticles : 0;

		shadowParticles = numParticles - offset;
		shadowParticlesOffset = offset;
	}

	if (g_buffers->activeIndices.size())
		DrawPoints(g_fluidRenderBuffers, shadowParticles, shadowParticlesOffset,
				radius, 2048, 1.0f, lightFov, g_lightPos, g_lightTarget,
				lightTransform, g_shadowMap, g_drawDensity);

	ShadowEnd();

	//----------------
	// lighting pass

	BindSolidShader(g_lightPos, g_lightTarget, lightTransform, g_shadowMap,
			0.0f, Vec4(g_clearColor, g_fogDistance));

	SetView(view, proj);
	SetCullMode(true);

	// When the benchmark measures async compute, we need a graphics workload that runs for a whole frame.
	// We do this by rerendering our simple graphics many times.
	int passes = g_increaseGfxLoadForAsyncComputeTesting ? 50 : 1;

	for (int i = 0; i != passes; i++) {

		DrawPlanes((Vec4*) g_params.planes, g_params.numPlanes,
				g_drawPlaneBias);

		if (g_drawMesh)
			DrawMesh(g_mesh, g_meshColor);

		DrawShapes();

		if (g_drawCloth && g_buffers->triangles.size())
			DrawCloth(&g_buffers->positions[0], &g_buffers->normals[0],
					g_buffers->uvs.size() ? &g_buffers->uvs[0].x : NULL,
					&g_buffers->triangles[0], g_buffers->triangles.size() / 3,
					g_buffers->positions.size(), 3, g_expandCloth);

		if (g_drawRopes) {
			for (size_t i = 0; i < g_ropes.size(); ++i)
				DrawRope(&g_buffers->positions[0], &g_ropes[i].mIndices[0],
						g_ropes[i].mIndices.size(),
						g_params.radius * 0.5f * g_ropeScale, i);
		}

		// give scene a chance to do custom drawing
		g_scenes[g_scene]->Draw(0);
	}
	UnbindSolidShader();

	// first pass of diffuse particles (behind fluid surface)
	if (g_drawDiffuse)
		RenderDiffuse(g_fluidRenderer, g_diffuseRenderBuffers, numDiffuse,
				radius * g_diffuseScale, float(g_screenWidth), aspect, fov,
				g_diffuseColor, g_lightPos, g_lightTarget, lightTransform,
				g_shadowMap, g_diffuseMotionScale, g_diffuseInscatter,
				g_diffuseOutscatter, g_diffuseShadow, false);

	if (g_drawEllipsoids) {
		// draw solid particles separately
		if (g_numSolidParticles && g_drawPoints)
			DrawPoints(g_fluidRenderBuffers, g_numSolidParticles, 0, radius,
					float(g_screenWidth), aspect, fov, g_lightPos,
					g_lightTarget, lightTransform, g_shadowMap, g_drawDensity);

		// render fluid surface
		RenderEllipsoids(g_fluidRenderer, g_fluidRenderBuffers,
				numParticles - g_numSolidParticles, g_numSolidParticles, radius,
				float(g_screenWidth), aspect, fov, g_lightPos, g_lightTarget,
				lightTransform, g_shadowMap, g_fluidColor, g_blur, g_ior,
				g_drawOpaque);

		// second pass of diffuse particles for particles in front of fluid surface
		if (g_drawDiffuse)
			RenderDiffuse(g_fluidRenderer, g_diffuseRenderBuffers, numDiffuse,
					radius * g_diffuseScale, float(g_screenWidth), aspect, fov,
					g_diffuseColor, g_lightPos, g_lightTarget, lightTransform,
					g_shadowMap, g_diffuseMotionScale, g_diffuseInscatter,
					g_diffuseOutscatter, g_diffuseShadow, true);
	} else {
		// draw all particles as spheres
		if (g_drawPoints) {
			int offset = g_drawMesh ? g_numSolidParticles : 0;

			if (g_buffers->activeIndices.size())
				DrawPoints(g_fluidRenderBuffers, numParticles - offset, offset,
						radius, float(g_screenWidth), aspect, fov, g_lightPos,
						g_lightTarget, lightTransform, g_shadowMap,
						g_drawDensity);
		}
	}

	GraphicsTimerEnd();
}

//float grab_z_pos_particle(int i) {
//	return z_pos_array[i];
//}
//float grab_y_pos_particle(int i) {
//	return y_pos_array[i];
//}
//float grab_x_pos_particle(int i) {
//	return x_pos_array[i];
//}

void RenderDebug() {

	for (int n = 0; n < 3548; n++) {
		x_pos_array[n] = g_buffers->positions[n].x;
		y_pos_array[n] = (g_buffers->positions[n].y - 0.23) * -100;
		z_pos_array[n] = g_buffers->positions[n].z;
	}

	if (g_mouseParticle != -1) {
		// draw mouse spring
		BeginLines();
		DrawLine(g_mousePos, Vec3(g_buffers->positions[g_mouseParticle]),
				Vec4(1.0f));
		EndLines();
	}

	// springs
	if (g_drawSprings) {
		Vec4 color;

		if (g_drawSprings == 1) {
			// stretch
			color = Vec4(0.0f, 0.0f, 1.0f, 0.8f);
		}
		if (g_drawSprings == 2) {
			// tether
			color = Vec4(0.0f, 1.0f, 0.0f, 0.8f);
		}

		BeginLines();

		int start = 0;

		for (int i = start; i < g_buffers->springLengths.size(); ++i) {
			if (g_drawSprings == 1 && g_buffers->springStiffness[i] < 0.0f)
				continue;
			if (g_drawSprings == 2 && g_buffers->springStiffness[i] > 0.0f)
				continue;

			int a = g_buffers->springIndices[i * 2];
			int b = g_buffers->springIndices[i * 2 + 1];

			//cout << i << " "<< g_buffers->positions[a] << " " << g_buffers->positions[a].x << endl;

			DrawLine(Vec3(g_buffers->positions[a]),
					Vec3(g_buffers->positions[b]), color);
		}

		EndLines();
	}

	// visualize contacts against the environment
	if (g_drawContacts) {
		const int maxContactsPerParticle = 6;

		NvFlexVector<Vec4> contactPlanes(g_flexLib,
				g_buffers->positions.size() * maxContactsPerParticle);
		NvFlexVector<Vec4> contactVelocities(g_flexLib,
				g_buffers->positions.size() * maxContactsPerParticle);
		NvFlexVector<int> contactIndices(g_flexLib,
				g_buffers->positions.size());
		NvFlexVector<unsigned int> contactCounts(g_flexLib,
				g_buffers->positions.size());

		NvFlexGetContacts(g_solver, contactPlanes.buffer,
				contactVelocities.buffer, contactIndices.buffer,
				contactCounts.buffer);

		// ensure transfers have finished
		contactPlanes.map();
		contactVelocities.map();
		contactIndices.map();
		contactCounts.map();

		BeginLines();

		//cout << g_buffers->positions[g_buffers->activeIndices[2]] << " NEXT " << g_buffers->positions[g_buffers->activeIndices[100]] << endl;

		for (int i = 0; i < int(g_buffers->activeIndices.size()); ++i) {
			const int contactIndex = contactIndices[g_buffers->activeIndices[i]];
			const unsigned int count = contactCounts[contactIndex];

			const float scale = 0.1f;

			for (unsigned int c = 0; c < count; ++c) {
				Vec4 plane = contactPlanes[contactIndex * maxContactsPerParticle
						+ c];

				DrawLine(
						Vec3(g_buffers->positions[g_buffers->activeIndices[i]]),
						Vec3(g_buffers->positions[g_buffers->activeIndices[i]])
								+ Vec3(plane) * scale,
						Vec4(0.0f, 1.0f, 0.0f, 0.0f));
				//cout << i << " " << contactIndex << " " << c << " " << count << " "  << " " << int(g_buffers->activeIndices.size()) << endl;

			}

		}

		EndLines();
	}

	if (g_drawBases) {
		for (int i = 0; i < int(g_buffers->rigidRotations.size()); ++i) {
			BeginLines();

			float size = 0.1f;

			for (int b = 0; b < 3; ++b) {
				Vec3 color;
				color[b] = 1.0f;

				Matrix33 frame(g_buffers->rigidRotations[i]);

				DrawLine(Vec3(g_buffers->rigidTranslations[i]),
						Vec3(
								g_buffers->rigidTranslations[i]
										+ frame.cols[b] * size),
						Vec4(color, 0.0f));
			}

			EndLines();
		}
	}

	if (g_drawNormals) {
		NvFlexGetNormals(g_solver, g_buffers->normals.buffer, NULL);

		BeginLines();

		for (int i = 0; i < g_buffers->normals.size(); ++i) {
			DrawLine(Vec3(g_buffers->positions[i]),
					Vec3(
							g_buffers->positions[i]
									- g_buffers->normals[i]
											* g_buffers->normals[i].w),
					Vec4(0.0f, 1.0f, 0.0f, 0.0f));
		}

		EndLines();
	}
}

void DrawShapes() {
	for (int i = 0; i < g_buffers->shapeFlags.size(); ++i) {
		const int flags = g_buffers->shapeFlags[i];

		// unpack flags
		int type = int(flags & eNvFlexShapeFlagTypeMask);
		//bool dynamic = int(flags&eNvFlexShapeFlagDynamic) > 0;

		Vec3 color = g_buffers->shapeColors[i];
//		Vec3 color = Vec3(0.9f);

		if (flags & eNvFlexShapeFlagTrigger) {
			color = Vec3(0.6f, 1.0, 0.6f);

			SetFillMode(true);
		}

		// render with prev positions to match particle update order
		// can also think of this as current/next
		const Quat rotation = g_buffers->shapePrevRotations[i];
		const Vec3 position = Vec3(g_buffers->shapePrevPositions[i]);

		NvFlexCollisionGeometry geo = g_buffers->shapeGeometry[i];

		if (type == eNvFlexShapeSphere) {
			Mesh* sphere = CreateSphere(20, 20, geo.sphere.radius);

			Matrix44 xform = TranslationMatrix(Point3(position))
					* RotationMatrix(Quat(rotation));
			sphere->Transform(xform);

			DrawMesh(sphere, Vec3(color));

			delete sphere;
		} else if (type == eNvFlexShapeCapsule) {
			Mesh* capsule = CreateCapsule(10, 20, geo.capsule.radius,
					geo.capsule.halfHeight);

			// transform to world space
			Matrix44 xform = TranslationMatrix(Point3(position))
					* RotationMatrix(Quat(rotation))
					* RotationMatrix(DegToRad(-90.0f), Vec3(0.0f, 0.0f, 1.0f));
			capsule->Transform(xform);

			DrawMesh(capsule, Vec3(color));

			delete capsule;
		} else if (type == eNvFlexShapeBox) {
			Mesh* box = CreateCubeMesh();

			Matrix44 xform = TranslationMatrix(Point3(position))
					* RotationMatrix(Quat(rotation))
					* ScaleMatrix(Vec3(geo.box.halfExtents) * 2.0f);
			box->Transform(xform);

			DrawMesh(box, Vec3(color));
			delete box;
		} else if (type == eNvFlexShapeConvexMesh) {
			if (g_convexes.find(geo.convexMesh.mesh) != g_convexes.end()) {
				GpuMesh* m = g_convexes[geo.convexMesh.mesh];

				if (m) {
					Matrix44 xform = TranslationMatrix(
							Point3(g_buffers->shapePositions[i]))
							* RotationMatrix(Quat(g_buffers->shapeRotations[i]))
							* ScaleMatrix(geo.convexMesh.scale);
					DrawGpuMesh(m, xform, Vec3(color));
				}
			}
		} else if (type == eNvFlexShapeTriangleMesh) {
			if (g_meshes.find(geo.triMesh.mesh) != g_meshes.end()) {
				GpuMesh* m = g_meshes[geo.triMesh.mesh];

				if (m) {
					Matrix44 xform = TranslationMatrix(Point3(position))
							* RotationMatrix(Quat(rotation))
							* ScaleMatrix(geo.triMesh.scale);
					DrawGpuMesh(m, xform, Vec3(color));
				}
			}
		} else if (type == eNvFlexShapeSDF) {
			if (g_fields.find(geo.sdf.field) != g_fields.end()) {
				GpuMesh* m = g_fields[geo.sdf.field];

				if (m) {
					Matrix44 xform = TranslationMatrix(Point3(position))
							* RotationMatrix(Quat(rotation))
							* ScaleMatrix(geo.sdf.scale);
					DrawGpuMesh(m, xform, Vec3(color));
				}
			}
		}
	}

	SetFillMode(g_wireframe);
}

void UpdateFrame(bool capture, char *path, Eigen::VectorXd action) {
	if (capture) {
		g_capture = true;
		g_ffmpeg = fopen(path, "wb");
	}

	static double lastTime;

	// real elapsed frame time
	double frameBeginTime = GetSeconds();

	g_realdt = float(frameBeginTime - lastTime);
	lastTime = frameBeginTime;

	// do gamepad input polling
	double currentTime = frameBeginTime;
	static double lastJoyTime = currentTime;

	//-------------------------------------------------------------------
	// Scene Update

	double waitBeginTime = GetSeconds();

	MapBuffers(g_buffers);

	double waitEndTime = GetSeconds();

	// Getting timers causes CPU/GPU sync, so we do it after a map
	float newSimLatency = NvFlexGetDeviceLatency(g_solver,
			&g_GpuTimers.computeBegin, &g_GpuTimers.computeEnd,
			&g_GpuTimers.computeFreq);
	float newGfxLatency = RendererGetDeviceTimestamps(&g_GpuTimers.renderBegin,
			&g_GpuTimers.renderEnd, &g_GpuTimers.renderFreq);
	(void) newGfxLatency;

	UpdateCamera();
	if (!g_pause || g_step) {
		UpdateMouse();
		UpdateWind();
//		Eigen::VectorXd action(4);
//		action << 0, 0, 0, 0;
		curr_state = UpdateControlScene(action);
//		cout<<"ASFASFASF0"<<endl;
	}

	//-------------------------------------------------------------------
	// Render
	double renderBeginTime = GetSeconds();

	if (visualize) {

		if (g_profile && (!g_pause || g_step)) {
			if (g_benchmark) {
				g_numDetailTimers = NvFlexGetDetailTimers(g_solver,
						&g_detailTimers);
			} else {
				memset(&g_timers, 0, sizeof(g_timers));
				NvFlexGetTimers(g_solver, &g_timers);
			}
		}

		StartFrame(Vec4(g_clearColor, 1.0f));

		// main scene render
		RenderScene();
		RenderDebug();

		EndFrame();

		// If user has disabled async compute, ensure that no compute can overlap
		// graphics by placing a sync between them
		if (!g_useAsyncCompute)
			NvFlexComputeWaitForGraphics(g_flexLib);
	}
	UnmapBuffers(g_buffers);

	// move mouse particle (must be done here as GetViewRay() uses the GL projection state)
	if (g_mouseParticle != -1) {
		Vec3 origin, dir;
		GetViewRay(g_lastx, g_screenHeight - g_lasty, origin, dir);

		g_mousePos = origin + dir * g_mouseT;
	}

	if (g_capture) {
		TgaImage img;
		img.m_width = g_screenWidth;
		img.m_height = g_screenHeight;
		img.m_data = new uint32_t[g_screenWidth * g_screenHeight];

		ReadFrame((int*) img.m_data, g_screenWidth, g_screenHeight);

//		fwrite(img.m_data, sizeof(uint32_t) * g_screenWidth * g_screenHeight, 1,
//				g_ffmpeg);
		TgaSave(g_ffmpeg, img, false);
		delete[] img.m_data;
	}

	double renderEndTime = GetSeconds();

	// if user requested a scene reset process it now
	if (g_resetScene) {
		Reset();
		g_resetScene = false;
	}

	//-------------------------------------------------------------------
	// Flex Update

	double updateBeginTime = GetSeconds();

	// send any particle updates to the solver
	NvFlexSetParticles(g_solver, g_buffers->positions.buffer, NULL);
	NvFlexSetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
	NvFlexSetPhases(g_solver, g_buffers->phases.buffer, NULL);
	NvFlexSetActive(g_solver, g_buffers->activeIndices.buffer, NULL);

	NvFlexSetActiveCount(g_solver, g_buffers->activeIndices.size());

	// allow scene to update constraints etc
	SyncScene();

	if (g_shapesChanged) {
		NvFlexSetShapes(g_solver, g_buffers->shapeGeometry.buffer,
				g_buffers->shapePositions.buffer,
				g_buffers->shapeRotations.buffer,
				g_buffers->shapePrevPositions.buffer,
				g_buffers->shapePrevRotations.buffer,
				g_buffers->shapeFlags.buffer,
				int(g_buffers->shapeFlags.size()));

		g_shapesChanged = false;
	}

	if (!g_pause || g_step) {
		// tick solver
		NvFlexSetParams(g_solver, &g_params);
		NvFlexUpdateSolver(g_solver, g_dt, g_numSubsteps, g_profile);

		g_frame++;
		g_step = false;
	}

	// read back base particle data
	// Note that flexGet calls don't wait for the GPU, they just queue a GPU copy
	// to be executed later.
	// When we're ready to read the fetched buffers we'll Map them, and that's when
	// the CPU will wait for the GPU flex update and GPU copy to finish.
	NvFlexGetParticles(g_solver, g_buffers->positions.buffer, NULL);
	NvFlexGetVelocities(g_solver, g_buffers->velocities.buffer, NULL);
	NvFlexGetNormals(g_solver, g_buffers->normals.buffer, NULL);

	// readback triangle normals
	if (g_buffers->triangles.size())
		NvFlexGetDynamicTriangles(g_solver, g_buffers->triangles.buffer,
				g_buffers->triangleNormals.buffer,
				g_buffers->triangles.size() / 3);

	// readback rigid transforms
	if (g_buffers->rigidOffsets.size())
		NvFlexGetRigids(g_solver, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				g_buffers->rigidRotations.buffer,
				g_buffers->rigidTranslations.buffer);

	if (!g_interop) {
		// if not using interop then we read back fluid data to host
		if (g_drawEllipsoids) {
			NvFlexGetSmoothParticles(g_solver,
					g_buffers->smoothPositions.buffer, NULL);
			NvFlexGetAnisotropy(g_solver, g_buffers->anisotropy1.buffer,
					g_buffers->anisotropy2.buffer,
					g_buffers->anisotropy3.buffer, NULL);
		}

		// read back diffuse data to host
		if (g_drawDensity)
			NvFlexGetDensities(g_solver, g_buffers->densities.buffer, NULL);

		if (GetNumDiffuseRenderParticles(g_diffuseRenderBuffers)) {
			NvFlexGetDiffuseParticles(g_solver,
					g_buffers->diffusePositions.buffer,
					g_buffers->diffuseVelocities.buffer,
					g_buffers->diffuseCount.buffer);
		}
	} else {
		// read back just the new diffuse particle count, render buffers will be updated during rendering
		NvFlexGetDiffuseParticles(g_solver, NULL, NULL,
				g_buffers->diffuseCount.buffer);
	}

	double updateEndTime = GetSeconds();

	//-------------------------------------------------------
	// Update the on-screen timers

	float newUpdateTime = float(updateEndTime - updateBeginTime);
	float newRenderTime = float(renderEndTime - renderBeginTime);
	float newWaitTime = float(waitBeginTime - waitEndTime);

	// Exponential filter to make the display easier to read
	const float timerSmoothing = 0.05f;

	g_updateTime =
			(g_updateTime == 0.0f) ?
					newUpdateTime :
					Lerp(g_updateTime, newUpdateTime, timerSmoothing);
	g_renderTime =
			(g_renderTime == 0.0f) ?
					newRenderTime :
					Lerp(g_renderTime, newRenderTime, timerSmoothing);
	g_waitTime =
			(g_waitTime == 0.0f) ?
					newWaitTime : Lerp(g_waitTime, newWaitTime, timerSmoothing);
	g_simLatency =
			(g_simLatency == 0.0f) ?
					newSimLatency :
					Lerp(g_simLatency, newSimLatency, timerSmoothing);

//	if (g_benchmark)
//		newScene = BenchmarkUpdate();

	// flush out the last frame before freeing up resources in the event of a scene change
	// this is necessary for d3d12
	if (visualize) {
		PresentFrame(g_vsync);
	}
	// if gui or benchmark requested a scene change process it now

	if (capture) {

		g_capture = false;
		fclose(g_ffmpeg);

		g_ffmpeg = nullptr;
	}

}

void ReshapeWindow(int width, int height) {
	if (!g_benchmark)
		printf("Reshaping\n");

	ReshapeRender(g_window);

	if (!g_fluidRenderer
			|| (width != g_screenWidth || height != g_screenHeight)) {
		if (g_fluidRenderer)
			DestroyFluidRenderer(g_fluidRenderer);
		g_fluidRenderer = CreateFluidRenderer(width, height);
	}

	g_screenWidth = width;
	g_screenHeight = height;
}

void InputArrowKeysDown(int key, int x, int y) {
	switch (key) {
	case SDLK_DOWN: {
		if (g_selectedScene < int(g_scenes.size()) - 1)
			g_selectedScene++;

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_selectedScene - 4) * 24, 0);
		break;
	}
	case SDLK_UP: {
		if (g_selectedScene > 0)
			g_selectedScene--;

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_selectedScene - 4) * 24, 0);
		break;
	}
	case SDLK_LEFT: {
		if (g_scene > 0)
			--g_scene;
		Init(g_scene);

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_scene - 4) * 24, 0);
		break;
	}
	case SDLK_RIGHT: {
		if (g_scene < int(g_scenes.size()) - 1)
			++g_scene;
		Init(g_scene);

		// update scroll UI to center on selected scene
		g_levelScroll = max((g_scene - 4) * 24, 0);
		break;
	}
	}
}

void InputArrowKeysUp(int key, int x, int y) {
}

bool InputKeyboardDown(unsigned char key, int x, int y) {
	if (key > '0' && key <= '9') {
		g_scene = key - '0' - 1;
		Init(g_scene);
		return false;
	}

	float kSpeed = g_camSpeed;

	switch (key) {
	case 'w': {
		g_camVel.z = kSpeed;
		break;
	}
	case 's': {
		g_camVel.z = -kSpeed;
		break;
	}
	case 'a': {
		g_camVel.x = -kSpeed;
		break;
	}
	case 'd': {
		g_camVel.x = kSpeed;
		break;
	}
	case 'q': {
		g_camVel.y = kSpeed;
		break;
	}
	case 'z': {
		//g_drawCloth = !g_drawCloth;
		g_camVel.y = -kSpeed;
		break;
	}

	case 'u': {
#ifndef ANDROID
		if (g_fullscreen) {
			SDL_SetWindowFullscreen(g_window, 0);
			ReshapeWindow(1280, 720);
			g_fullscreen = false;
		} else {
			SDL_SetWindowFullscreen(g_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
			g_fullscreen = true;
		}
#endif
		break;
	}
	case 'r': {
		g_resetScene = true;
		break;
	}
	case 'y': {
		g_wavePool = !g_wavePool;
		break;
	}
	case 'c': {
		break;
	}
	case 'p': {
		g_pause = !g_pause;
		break;
	}
	case 'o': {
		g_step = true;
		break;
	}
	case 'h': {
		g_showHelp = !g_showHelp;
		break;
	}
	case 'e': {
		g_drawEllipsoids = !g_drawEllipsoids;
		break;
	}
	case 't': {
		g_drawOpaque = !g_drawOpaque;
		break;
	}
	case 'v': {
		g_drawPoints = !g_drawPoints;
		break;
	}
	case 'f': {
		g_drawSprings = (g_drawSprings + 1) % 3;
		break;
	}
	case 'i': {
		g_drawDiffuse = !g_drawDiffuse;
		break;
	}
	case 'm': {
		g_drawMesh = !g_drawMesh;
		break;
	}
	case 'n': {
		g_drawRopes = !g_drawRopes;
		break;
	}
	case 'j': {
		g_windTime = 0.0f;
		g_windStrength = 1.5f;
		g_windFrequency = 0.2f;
		break;
	}
	case '.': {
		g_profile = !g_profile;
		break;
	}
	case 'g': {
		if (g_params.gravity[1] != 0.0f)
			g_params.gravity[1] = 0.0f;
		else
			g_params.gravity[1] = -9.8f;

		break;
	}
	case '-': {
		if (g_params.numPlanes)
			g_params.numPlanes--;

		break;
	}
	case ' ': {
		g_emit = !g_emit;
		break;
	}
	case ';': {
		g_debug = !g_debug;
		break;
	}
	case 13: {
		g_scene = g_selectedScene;
		Init(g_scene);
		break;
	}
	case 27: {
		// return quit = true
		return true;
	}
	};

	g_scenes[g_scene]->KeyDown(key);

	return false;
}

void InputKeyboardUp(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
	case 's': {
		g_camVel.z = 0.0f;
		break;
	}
	case 'a':
	case 'd': {
		g_camVel.x = 0.0f;
		break;
	}
	case 'q':
	case 'z': {
		g_camVel.y = 0.0f;
		break;
	}
	};
}

void MouseFunc(int b, int state, int x, int y) {
	switch (state) {
	case SDL_RELEASED: {
		g_lastx = x;
		g_lasty = y;
		g_lastb = -1;

		break;
	}
	case SDL_PRESSED: {
		g_lastx = x;
		g_lasty = y;
		g_lastb = b;
#ifdef ANDROID
		extern void setStateLeft(bool bLeftDown);
		setStateLeft(false);
#else
		if ((SDL_GetModState() & KMOD_LSHIFT) && g_lastb == SDL_BUTTON_LEFT) {
			// record that we need to update the picked particle
			g_mousePicked = true;
		}
#endif
		break;
	}
	};
}

void MousePassiveMotionFunc(int x, int y) {
	g_lastx = x;
	g_lasty = y;
}

void MouseMotionFunc(unsigned state, int x, int y) {
	float dx = float(x - g_lastx);
	float dy = float(y - g_lasty);

	g_lastx = x;
	g_lasty = y;

	if (state & SDL_BUTTON_RMASK) {
		const float kSensitivity = DegToRad(0.1f);
		const float kMaxDelta = FLT_MAX;

		g_camAngle.x -= Clamp(dx * kSensitivity, -kMaxDelta, kMaxDelta);
		g_camAngle.y -= Clamp(dy * kSensitivity, -kMaxDelta, kMaxDelta);
	}
}

bool g_Error = false;

void ErrorCallback(NvFlexErrorSeverity severity, const char* msg,
		const char* file, int line) {
	printf("Flex: %s - %s:%d\n", msg, file, line);
	g_Error = (severity == eNvFlexLogError);
	//assert(0); asserts are bad for TeamCity
}

void SDLInit(const char* title) {

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)	// Initialize SDL's Video subsystem and game controllers
		printf("Unable to initialize SDL");

	unsigned int flags = SDL_WINDOW_RESIZABLE;
#if !FLEX_DX
	if (g_graphics == 0) {
		SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
		flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL;
	}
#endif

	g_window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED,
	SDL_WINDOWPOS_CENTERED, g_screenWidth, g_screenHeight, flags);

	g_windowId = SDL_GetWindowID(g_window);
}

bool SDLMain() {

	bool quit = false;
	if (visualize) {
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
			case SDL_QUIT:
				quit = true;
				break;

			case SDL_KEYDOWN:
				if (e.key.keysym.sym < 256
						&& (e.key.keysym.mod == KMOD_NONE
								|| (e.key.keysym.mod & KMOD_NUM)))
					quit = InputKeyboardDown(e.key.keysym.sym, 0, 0);
				InputArrowKeysDown(e.key.keysym.sym, 0, 0);
				break;

			case SDL_KEYUP:
				if (e.key.keysym.sym < 256
						&& (e.key.keysym.mod == 0
								|| (e.key.keysym.mod & KMOD_NUM)))
					InputKeyboardUp(e.key.keysym.sym, 0, 0);
				InputArrowKeysUp(e.key.keysym.sym, 0, 0);
				break;

			case SDL_MOUSEMOTION:
				if (e.motion.state)
					MouseMotionFunc(e.motion.state, e.motion.x, e.motion.y);
				else
					MousePassiveMotionFunc(e.motion.x, e.motion.y);
				break;

			case SDL_MOUSEBUTTONDOWN:
			case SDL_MOUSEBUTTONUP:
				MouseFunc(e.button.button, e.button.state, e.motion.x,
						e.motion.y);
				break;

			case SDL_WINDOWEVENT:
				if (e.window.windowID == g_windowId) {
					if (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
						ReshapeWindow(e.window.data1, e.window.data2);
				}
				break;

			case SDL_WINDOWEVENT_LEAVE:
				g_camVel = Vec3(0.0f, 0.0f, 0.0f);
				break;
			}

		}
	}
	return quit;

}

void initialize() {


	g_scenes.push_back(
				new GranularSweepShaping("Granular Reshaping"));
//	g_scenes.push_back(new PlasticBodyReshaping("Plastic Reshaping"));
	g_scenes.push_back(new PlasticSpringShaping("Plastic Spring Reshaping"));

	g_scenes.push_back(new PlasticSpringFlipping("Plastic Flipping"));
	g_scenes.push_back(new GranularFlipping("Granular Flipping"));

	g_scenes.push_back(
			new PlasticSpringShapingManualControl(
					"Plastic Reshaping Using Springs Single Instance"));
	g_scenes.push_back(
			new GranularSweepShapingManualControl(
					"Granular Sweep Conrollable Ghost Single Instance"));

	g_scenes.push_back(new PlasticSpringFlippingManualControl("Plastic Flipping Single Instance"));

//	g_scenes.push_back(
//			new GooShapingManualControl("Goo Reshaping Single Instance"));
//	g_scenes.push_back(
//			new GooShapingExpManualControl(
//					"Goo Reshaping Expensive Single Instance"));
//	g_scenes.push_back(new GooShaping("Goo Reshaping"));
//
//
//

//    g_scenes.push_back(new ForceField("Force Field"));

	// init graphics
	RenderInitOptions options;

//    cout << "*********** Starting FleX with Python ************ " << endl;

	CreateDemoContext(g_graphics);

	std::string str;

	str = "Flex Demo (Compute: CUDA) ";

	str += "(Graphics: OpenGL)";
	if (visualize) {

		const char* title = str.c_str();
		RenderInitOptions options;
		SDLInit(title);
		options.window = g_window;
		options.numMsaaSamples = g_msaaSamples;
		options.asyncComputeBenchmark = g_asyncComputeBenchmark;
		options.defaultFontHeight = -1;
		options.fullscreen = g_fullscreen;
		InitRender(options);

		if (g_fullscreen)
			SDL_SetWindowFullscreen(g_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
		ReshapeWindow(g_screenWidth, g_screenHeight);

		g_shadowMap = ShadowCreate();

	}

	NvFlexInitDesc desc;
	desc.deviceIndex = g_device;
	desc.enableExtensions = g_extensions;
	desc.renderDevice = 0;
	desc.renderContext = 0;
	desc.computeContext = 0;
	desc.computeType = eNvFlexCUDA;

	// Init Flex library, note that no CUDA methods should be called before this
	// point to ensure we get the device context we want
	g_flexLib = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);

	if (g_Error || g_flexLib == NULL) {
		printf("Could not initialize Flex, exiting.\n");
		exit(-1);
	}

	// store device name
	strcpy(g_deviceName, NvFlexGetDeviceName(g_flexLib));
	printf("Compute Device: %s\n\n", g_deviceName);

	if (g_benchmark)
		g_scene = BenchmarkInit();

	// create shadow maps

	// init default scene
	StartGpuWork();
	Init(true);
	EndGpuWork();

}

int destroy_scene() {

	cout << "C++ Destroying Scene!" << endl;

	if (g_fluidRenderer)
		DestroyFluidRenderer(g_fluidRenderer);

	DestroyFluidRenderBuffers(g_fluidRenderBuffers);
	DestroyDiffuseRenderBuffers(g_diffuseRenderBuffers);

	ShadowDestroy(g_shadowMap);

	Shutdown();
	DestroyRender();

	SDL_DestroyWindow(g_window);
	SDL_Quit();

	return 0;
}

void setSceneRandSeed(int seed) {
	g_scenes[g_scene]->setSceneSeed(seed);
}

float getDt() {
	return g_dt;
}
void setDt(float dt) {
	g_dt = dt;
}
int getNumParticles() {
	return g_buffers->positions.size();
}

int getNumInstances() {
	return g_scenes[g_scene]->getNumInstances();
}

void setVisualization(bool vis) {
	visualize = vis;
}

Eigen::MatrixXd getAllCenters() {
	return g_scenes[g_scene]->getAllCenters();
}
Eigen::MatrixXd getState() {

//	return g_scenes[g_scene]->getState();
	return curr_state;
}

void chooseScene(int scene) {
	g_scene = scene;
}

void setController(Eigen::MatrixXd controllerConfig) {
	g_scenes[g_scene]->setControllerInit(controllerConfig);
	MapBuffers(g_buffers);
	curr_state = g_scenes[g_scene]->getState();
	UnmapBuffers(g_buffers);
}

void setAuxInfo(Eigen::MatrixXd info){
	g_scenes[g_scene]->UpdateGUI(info);
}

Eigen::MatrixXd getParticleDensity(Eigen::MatrixXd particles, int resolution,
		float width,float mapHalfExtent) {
	Eigen::MatrixXd density(resolution, resolution);
	density.setZero();
	float dx = 1.0f / resolution;
	float inv_dx = 1.0f / dx;
	for (int i = 0; i < particles.rows(); i++) {
		if (abs(particles(i, 0)) <= mapHalfExtent && abs(particles(i, 1)) <= mapHalfExtent) {

			Eigen::Vector2d base_coord(
					((particles(i, 0) + mapHalfExtent) / (2*mapHalfExtent) * inv_dx - 0.5),
					((particles(i, 1) + mapHalfExtent) / (2*mapHalfExtent) * inv_dx - 0.5));

			Eigen::Vector2d fx(base_coord[0] - (int) base_coord[0],
					base_coord[1] - (int) base_coord[1]);

			int gridWidth = ceil(width);
			for (int j = -gridWidth + 1; j < gridWidth + 1; j++) {
				for (int k = -gridWidth + 1; k < gridWidth + 1; k++) {
					Eigen::Vector2d neighbour(j, k);
					Eigen::Vector2d dpos(0, 0);
					dpos = neighbour - fx;

					float wx = 0, wy = 0;

					wx = width - abs(dpos[0]);

					if (wx < 0) {
						wx = 0;
					}
					wy = width - abs(dpos[1]);
					if (wy < 0) {
						wy = 0;
					}

					if ((int) base_coord[1] + j < resolution
							&& (int) base_coord[0] + k < resolution
							&& (int) base_coord[1] + j >= 0
							&& (int) base_coord[0] + k >= 0) {
						density((int) base_coord[1] + j,
								(int) base_coord[0] + k) += sqrt(
								wx * wx + wy * wy);
					}

				}
			}
		}

	}
	return density;

}

Eigen::Vector3d getParticleAngularVelocity(Eigen::MatrixXd allPartPos, Eigen::MatrixXd allPartVel){
	Eigen::Vector3d angVel(0);
	if(allPartPos.rows()>1){

		Eigen::Vector3d COM(0);
		for (int i = 0; i < allPartPos.rows(); i++) {
			Eigen::Vector3d partPos(allPartPos.row(i));
			COM+=partPos;
		}
		COM/=(1.0f*allPartPos.rows());

		for (int i = 0; i < allPartVel.rows(); i++) {
			Eigen::Vector3d partPos(allPartPos.row(i));

			Eigen::Vector3d pattVel(allPartVel.row(i));

			Eigen::Vector3d r1(partPos-COM);

			Eigen::Vector3d axisOfRotation(r1.cross(pattVel));
			if(r1.norm()<0.001f){
				axisOfRotation/= r1.norm()*r1.norm();
			}

			angVel+=axisOfRotation;
		}
		angVel/=(1.0f*allPartPos.rows());
	}


	return angVel;

}
//Eigen::Vector3d getParticleAngularVelocity(Eigen::MatrixXd prevParticles, Eigen::MatrixXd currParticles){
//	Eigen::Vector3d COM1(0);
//	Eigen::Vector3d COM2(0);
//	for (int i = 0; i < prevParticles.rows(); i++) {
//		Eigen::Vector3d prevPartPos(prevParticles.row(i));
//		Eigen::Vector3d currPartPos(currParticles.row(i));
//		COM1+=prevPartPos;
//		COM2+=currPartPos;
//	}
//	COM1/=(1.0f*prevParticles.rows());
//	COM2/=(1.0f*prevParticles.rows());
//	Eigen::Vector3d angVel(0);
//	for (int i = 0; i < prevParticles.rows(); i++) {
//		Eigen::Vector3d prevPartPos(prevParticles.row(i));
//		Eigen::Vector3d currPartPos(currParticles.row(i));
//
//		Eigen::Vector3d r1(prevPartPos-COM1);
//		Eigen::Vector3d r2(currPartPos-COM2);
//
//		r1.normalize();
//		r2.normalize();
//		Eigen::Vector3d axisOfRotation(r1.cross(r2));
//
//		angVel+=axisOfRotation;
//	}
//	angVel/=(1.0f*prevParticles.rows());
//
//
//
//	return angVel;
//
//}

Eigen::MatrixXd getParticleHeightMap(Eigen::MatrixXd particles, Eigen::VectorXd heights,int resolution,
		float width,float mapHalfExtent) {
	Eigen::MatrixXd heightMap(resolution, resolution);
	Eigen::MatrixXd weights (resolution,resolution);
	heightMap.setZero();
	weights.setOnes();
	float dx = 1.0f / resolution;
	float inv_dx = 1.0f / dx;
	for (int i = 0; i < particles.rows(); i++) {
		if (abs(particles(i, 0)) <= mapHalfExtent && abs(particles(i, 1)) <= mapHalfExtent) {

			Eigen::Vector2d base_coord(
					((particles(i, 0) + mapHalfExtent) / (2*mapHalfExtent) * inv_dx - 0.5),
					((particles(i, 1) + mapHalfExtent) / (2*mapHalfExtent) * inv_dx - 0.5));

//			heightMap((int) base_coord[1],(int) base_coord[0]) += heights[i];
//			weights((int) base_coord[1],(int) base_coord[0]) += 1.0;
			Eigen::Vector2d fx(base_coord[0] - (int) base_coord[0],
					base_coord[1] - (int) base_coord[1]);

			int gridWidth = ceil(width);
			for (int j = -gridWidth + 1; j < gridWidth + 1; j++) {
				for (int k = -gridWidth + 1; k < gridWidth + 1; k++) {
					Eigen::Vector2d neighbour(j, k);
					Eigen::Vector2d dpos(0, 0);
					dpos = neighbour - fx;

					float wx = 0, wy = 0;

					wx = width - abs(dpos[0]);

					if (wx < 0) {
						wx = 0;
					}
					wy = width - abs(dpos[1]);
					if (wy < 0) {
						wy = 0;
					}

					if ((int) base_coord[1] + j < resolution
							&& (int) base_coord[0] + k < resolution
							&& (int) base_coord[1] + j >= 0
							&& (int) base_coord[0] + k >= 0) {
						heightMap((int) base_coord[1] + j,
								(int) base_coord[0] + k) += heights[i]*sqrt(
								wx * wx + wy * wy);
						weights((int) base_coord[1] + j,
								(int) base_coord[0] + k) += sqrt(
										wx * wx + wy * wy);
					}

				}
			}
		}
	}
	return heightMap.cwiseQuotient(weights);

}
bool simulateKSteps(bool capture, char *path, Eigen::VectorXd action, int k) {
	bool done = false;
	for (int i = 0; i < k; i++) {
		UpdateFrame(capture, path, action);
		capture = false;
		done = SDLMain();
		if (done) {
			break;
		}
	}
	return done;
}

void setGoal(Eigen::MatrixXd goals) {
	g_scenes[g_scene]->setGoal(goals);
}
void setInitClusterParam(Eigen::MatrixXd clusterParam){
	g_scenes[g_scene]->setInitClusterParam(clusterParam);
}
void setMapHalfExtent(float mapHalfExtent){
	g_scenes[g_scene]->setMapHalfExtent(mapHalfExtent);
};
