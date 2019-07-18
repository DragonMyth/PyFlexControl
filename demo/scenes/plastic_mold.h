#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
//

class PlasticMold: public Scene {
public:

	PlasticMold(const char* name) :
			Scene(name) {
	}

	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;

	float kp_pos = 0.5;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	int particleDim = 30;
	int numSceneDim = 5;
	int seed = -1;
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

				float initAngOnCirc = Randf(-EIGEN_PI, EIGEN_PI);
				Eigen::Vector2d randPos(cosf(initAngOnCirc) * 3,
						sinf(initAngOnCirc) * 3);

//				Eigen::Vector2d randPos;
//				randPos.setRandom();
//				randPos = randPos*1;
//				Eigen::VectorXd randRot(1);
//				randRot.setRandom();
//				randRot *= EIGEN_PI/2;

				currPos = center + Vec3(randPos[0], 0, randPos[1]);

				currRot = QuatFromAxisAngle(Vec3(0, 1, 0),
						initAngOnCirc + EIGEN_PI / 2);

//				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));
				currVel = Vec3(0, 0, 0);
				currAngVel = 0;

				AddBox(Vec3(0.7, 0.5, 0.03), currPos, currRot);

				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);

			}

		}
//		Vec3 center = Vec3(0, 0, 0);
//		CreateParticleGrid(center + Vec3(0, 1, 0), 50, 1, 50, interRadius,
//				Vec3(0, 0, 0), 1, false, 0.0f,
//				NvFlexMakePhase(0, eNvFlexPhaseSelfCollide), 0.001f);

		g_numSubsteps = 1;
		g_params.radius = radius;
		g_params.staticFriction = 10.5f;
		g_params.dynamicFriction = 1.2f;
		g_params.viscosity = 0.0f;
		g_params.numIterations = 3;
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

				Vec3 targetPos = centers[i]
						+ Vec3(action(i * 4), 0, action(i * 4 + 1));
				Vec2 targetRotVec = Vec2(action(i * 4 + 2), action(i * 4 + 3));

				float currCosHalfAng = currRots[i].w;
				float currSinHalfAng = currRots[i].y;

				float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
				float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

				float angDiff = VectorToAngle(targetRotVec)
						- VectorToAngle(Vec2(currCosAng, currSinAng));

				if (angDiff > EIGEN_PI) {
					angDiff -= 2 * EIGEN_PI;
				} else if (angDiff <= -EIGEN_PI) {
					angDiff += 2 * EIGEN_PI;
				}

				Vec3 posDiff = targetPos - currPoses[i];

				Vec3 force = posDiff * kp_pos - currVels[i] * kd_pos;
				float torque = angDiff * kp_rot - currAngVels[i] * kd_rot;

				currVels[i] += force * g_dt;
				currAngVels[i] += torque * g_dt;

				Vec3 newPos = Vec3(currPoses[i].x, currPoses[i].y,
						currPoses[i].z) + currVels[i] * g_dt;
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
//		cout<<getState()<<endl;
	}

	void setSceneSeed(int seed) {
		this->seed = seed;
	}
	Eigen::MatrixXd getState() {
		using namespace Eigen;
		int numPart = g_buffers->positions.size();

		int numBars = numSceneDim * numSceneDim;
		//The last four rows are the translational and rotational position and velocity for the moving bar
		MatrixXd state(numPart + 4 * numBars, 2);
		cout << state.rows() << endl;
		state.setZero();

		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
			float currCosHalfAng = currRots[i].w;
			float currSinHalfAng = currRots[i].y;
			float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
			float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

			int numPartInScene = numPart / numBars;

			for (int j = 0; j < numPartInScene; j++) {
				state.row(i * (numPartInScene + 4) + j + 4) = Vector2d(
						g_buffers->positions[i * numPartInScene + j].x,
						g_buffers->positions[i * numPartInScene + j].z)
						- Vector2d(cent.x, cent.z);
			}
			state.row(i * (numPartInScene + 4)) = Vector2d(currPoses[i].x,
					currPoses[i].z) - Vector2d(cent.x, cent.z);
			;
			state.row(i * (numPartInScene + 4) + 1) = Vector2d(currCosAng,
					currSinAng);
			state.row(i * (numPartInScene + 4) + 2) = Vector2d(currVels[i].x,
					currVels[0].z);
			state.row(i * (numPartInScene + 4) + 3) = Vector2d(
					cos(currAngVels[0]), sin(currAngVels[0]));

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
