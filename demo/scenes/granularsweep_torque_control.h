#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
//

class GranularSweepTorqueControl: public Scene {
public:

	GranularSweepTorqueControl(const char* name) :
			Scene(name) {
	}

	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;

	int particleDim = 30;
	int numSceneDim = 5;
	int seed = -1;

	float maxVel = 3;
	float maxAngVel = EIGEN_PI;
	vector<Vec3> centers;

	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {
		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		if (seed != -1) {
			srand(seed);
		}
		// granular pile
		float radius = 0.05f;

		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {
				int group = i * 5 + j;
				int phase = NvFlexMakePhase(group, eNvFlexPhaseSelfCollide);
				Vec3 center = Vec3(i * 30, 0, j * 30);
				centers.push_back(center);
//				CreateParticleGrid(center + Vec3(0, radius, 0), particleDim, 1,
//						particleDim, radius, Vec3(0, 0, 0), 1, false, 0.0f,
//						phase, 0.005f);
				Vec2 xRange = Vec2(-2, 2);
				Vec2 zRange = Vec2(-2, 2);
				Vec2 gap = Vec2(particleDim, particleDim);
				CreateGranularGrid(center, xRange, zRange, radius, gap,
						Vec3(0, 0, 0), 1, false, 0.0f, phase, 0.2f);

				Vec3 currPos;
				Quat currRot;

				Vec3 currVel;
				float currAngVel;

				Eigen::Vector2d randPos;
				randPos.setRandom();
				randPos = randPos * 1;
				Eigen::VectorXd randRot(1);
				randRot.setRandom();
				randRot *= EIGEN_PI / 2;
				currPos = center + Vec3(randPos[0], 0, randPos[1]);

				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));
				currVel = Vec3(0, 0, 0);
				currAngVel = 0;

				AddBox(Vec3(0.7, 0.5, 0.03), currPos, currRot);

				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);

			}

		}

		g_numSubsteps = 5;
		g_params.radius = radius;
		g_params.staticFriction = 10.5f;
		g_params.dynamicFriction = 1.2f;
		g_params.viscosity = 0.0f;
		g_params.numIterations = 5;
		g_params.particleCollisionMargin = g_params.radius * 0.05f;	// 5% collision margin
		g_params.sleepThreshold = g_params.radius * 0.25f;
		g_params.shockPropagation = 6.f;
		g_params.restitution = 0.2f;
		g_params.relaxationFactor = 1.f;
		g_params.damping = 0.14f;
		g_params.numPlanes = 1;

		// draw options
		g_drawPoints = true;
		g_drawMesh = false;
		g_warmup = false;

		// hack, change the color of phase 0 particles to 'sand'		
//		g_colors[0] = Colour(0.805f, 0.702f, 0.401f);

		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;

		//		action = VectorXd(4);
		//		action << 0, 0, 0, 1;
//		cout << "Num of Particles: " << g_buffers->positions[0] << endl;
//
//		cout << "Angle Diff: " << angDiff << endl;
//		cout << "Position Diff: " << posDiff.x << " " << posDiff.y << " "
//				<< posDiff.z << endl;

		if (g_frame > 40) {
			ClearShapes();

			for (int i = 0; i < centers.size(); i++) {

				Vec3 force = Vec3(action(i * 3), 0, action(i * 3 + 1));
				float torque = action(i * 3 + 2);

				currVels[i] += force * g_dt;

				currVels[i].x = maxf(minf(currVels[i].x, maxVel), -maxVel);
				currVels[i].y = maxf(minf(currVels[i].y, maxVel), -maxVel);
				currVels[i].z = maxf(minf(currVels[i].z, maxVel), -maxVel);

				currAngVels[i] = torque;
				currAngVels[i] = maxf(minf(currAngVels[i], maxAngVel),
						-maxAngVel);

				Vec3 newPos;
				newPos = Vec3(currPoses[i].x, currPoses[i].y, currPoses[i].z)
						+ currVels[i] * g_dt;

				newPos.x = maxf(minf(newPos.x-centers[i].x, 4), -4)+centers[i].x;
				newPos.y = maxf(minf(newPos.y-centers[i].y, 4), -4)+centers[i].y;
				newPos.z = maxf(minf(newPos.z-centers[i].z, 4), -4)+centers[i].z;

				Quat newRot = currRots[i]
						* QuatFromAxisAngle(Vec3(0, 1, 0),
								currAngVels[i] * g_dt);

				currPoses[i] = newPos;
				currRots[i] = newRot;

				AddBox(Vec3(0.7, 0.5, 0.03), newPos, newRot);
			}
			UpdateShapes();

		}
		return getState();
	}

	void setSceneSeed(int seed) {
		this->seed = seed;
	}
	Eigen::MatrixXd getState() {
		using namespace Eigen;
		int numPart = g_buffers->positions.size();

		int numBars = numPart / (particleDim * particleDim);
		//The last four rows are the translational and rotational position and velocity for the moving bar
		MatrixXd state(numPart + 4 * numBars, 2);
		state.setZero();

		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
			int numPartInScene = particleDim * particleDim;

			for (int j = 0; j < numPartInScene; j++) {
				state.row(i * numPartInScene + j) = Vector2d(
						g_buffers->positions[i * numPartInScene + j].x,
						g_buffers->positions[i * numPartInScene + j].z)
						- Vector2d(cent.x, cent.z);
			}

		}

		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
			float currCosHalfAng = currRots[i].w;
			float currSinHalfAng = currRots[i].y;
			float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
			float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

			state.row(numPart + i * 4) = Vector2d(currPoses[i].x,
					currPoses[i].z) - Vector2d(cent.x, cent.z);
			;
			state.row(numPart + i * 4 + 1) = Vector2d(currCosAng, currSinAng);
			state.row(numPart + i * 4 + 2) = Vector2d(currVels[i].x,
					currVels[0].z);
			state.row(numPart + i * 4 + 3) = Vector2d(cos(currAngVels[0]),
					sin(currAngVels[0]));
		}
		return state;

	}

	int getNumInstances() {
		return numSceneDim * numSceneDim;
	}

	virtual Eigen::MatrixXd getAllCenters() {
		using namespace Eigen;
		int numInstances = numSceneDim * numSceneDim;
		MatrixXd allCenters(numInstances, 3);
		allCenters.setZero();
		for (int i = 0; i < numInstances; i++) {
			Vector3d centerVec(centers[i].x, centers[i].y, centers[i].z);
			allCenters.row(i) = centerVec;
		}
		return allCenters;
	}
};
