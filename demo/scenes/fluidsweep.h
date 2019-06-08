#include "../helpers.h"
//#include <Eigen/Dense>
class FluidSweep: public Scene {
public:

	FluidSweep(const char* name) :
			Scene(name) {
	}

	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;
	Vec3 barDim = Vec3(0.15, 0.2, 0.01);
	float kp_pos = 0.5;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	int particleDim = 100;
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
		float radius = 0.02f;

		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {
				int group = i * 5 + j;
				int phase = NvFlexMakePhase(group,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid);
				Vec3 center = Vec3(i * 10, 0, j * 10);
				centers.push_back(center);
				int thickness = 1;
				Vec2 xRange = Vec2(-0.5, 0.5);
				Vec2 yRange = Vec2(radius + 0.0001,
						thickness * radius + 0.0001);

				Vec2 zRange = Vec2(-0.5, 0.5);
				Vec3 partDims = Vec3(particleDim, thickness, particleDim);

				CreateGooGrid(center, xRange, yRange, zRange, radius, partDims,
						Vec3(0, 0, 0), 1, false, 0.0f, phase, 0.005f);

				Vec3 currPos;
				Quat currRot;

				Vec3 currVel;
				float currAngVel;

				Eigen::Vector2d randPos;
				randPos.setRandom();
				randPos = randPos * 0.8f;
				Eigen::VectorXd randRot(1);
				randRot.setRandom();
				randRot *= EIGEN_PI;
				currPos = center + Vec3(randPos[0], 0, randPos[1]);
				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));
				currVel = Vec3(0, 0, 0);
				currAngVel = 0;

				AddBox(barDim, currPos, currRot);

				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);

			}

		}

		g_numSubsteps = 3;
		g_params.numIterations = 3;
		g_params.radius = radius;

		g_params.staticFriction = 1.2f;
		g_params.dynamicFriction = 0.3f;

		g_params.particleCollisionMargin = g_params.radius * 0.01f;	// 5% collision margin
		g_params.sleepThreshold = g_params.radius * 0.25f;
		g_params.shockPropagation = 6.f;
		g_params.damping = 0.14f;
		g_params.numPlanes = 1;

		g_params.vorticityConfinement = 0.0f;
		g_params.fluidRestDistance = g_params.radius * 0.55f;
		g_params.smoothing = 0.7f;
		g_params.relaxationFactor = 1.f;
		g_params.restitution = 0.0f;
		g_params.collisionDistance = 0.001f;

		g_fluidColor = Vec4(0.0f, 0.8f, 0.2f, 1.0f);
		//g_fluidColor = Vec4(0.7f, 0.6f, 0.6f, 0.2f);

		g_params.viscosity = 100.0f;
		g_params.adhesion = 0.0f;
		g_params.cohesion = 0.3f;
		g_params.surfaceTension = 5.0f;
//		g_params.gravity[1] = -0.f;

		// draw options
		g_drawPoints = true;
//		g_drawFluids = false;
		g_drawMesh = false;
		g_drawEllipsoids = true;
		g_warmup = true;
		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;

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

				AddBox(barDim, newPos, newRot);
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
		//		cout << "HERERE" << endl;
		using namespace Eigen;
		int numPart = g_buffers->positions.size();
		//		cout<<"Num Parts: "<<numPart<<endl;
		//		cout<<"Num Expect: "<<particleDim*particleDim*numSceneDim*numSceneDim<<endl;
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
