#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
using namespace std;

//

class GooShapingManualControl: public Scene {
public:

	GooShapingManualControl(const char* name) :
			Scene(name) {
	}

	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;

	float kp_pos = 0.3;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	int numSceneDim = 1;
	int seed = -1;
	Eigen::MatrixXd goalPos;

	vector<Vec3> centers;
	Vec3 barDim = Vec3(1.5, 1, 0.03);

	int dimx = 16;
	int dimy = 5;
	int dimz = 16;

	float stiffness = 1.0f;

	float radius = 0.3f;
	int actionDim = 5;
	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {

		g_lightDistance *= 100.5f;

		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		Vec3 lower, upper;
		float restDist = radius*0.5;
		for (int i = 0; i < numSceneDim; i++) {
			for (int j = 0; j < numSceneDim; j++) {

				Vec3 center = Vec3(i * 15, 0, j * 15);
				int group = centers.size();
				int channel = eNvFlexPhaseShapeChannel0;

				int phase = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid, channel);

				CreateParticleGrid(center + Vec3(0, 0.5*radius, 0), dimx, dimy, dimz,
						restDist, Vec3(0.0f), 1.0f, false, 0.0f, phase, 0.0f);

				if (i == 0 && j == 0) {
					GetParticleBounds(lower, upper);
					cout << "Lower: " << lower.x << " " << lower.y << " "
							<< lower.z << endl;
					cout << "Upper: " << upper.x << " " << upper.y << " "
							<< upper.z << endl;
				}

				center += (upper - lower) / 2;
				center[1] = 0;
				centers.push_back(center);

				Vec3 currPos = center + Vec3(0,0,0);
				Quat currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0);
				Vec3 currVel = Vec3(0, 0, 0);
				float currAngVel = 0;
				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);
				barDim = Vec3(1.5, 1, 0.01);

				group++;

			}

		}

		g_params.staticFriction = 1.2f;
		g_params.dynamicFriction = 0.3f;
		g_params.sleepThreshold = g_params.radius * 0.25f;
		g_params.damping = 0.5f;
		g_params.dissipation=0.5f;
		g_params.radius = radius;

		g_params.vorticityConfinement = 0.0f;
		g_params.fluidRestDistance = restDist;
		g_params.smoothing = 0.5f;
		g_params.relaxationFactor = 1.f;
		g_params.restitution = 0.5f;
		g_params.collisionDistance = 0.1f * radius;

		g_params.viscosity = 100.0f;
		g_params.cohesion = 0.1f;
		g_params.adhesion = 0.0f;
		g_params.surfaceTension = 0;

//		g_params.gravity[1] = 0.0f;
		g_params.numIterations = 3;

		g_numSubsteps = 2;

		// draw options
		g_drawPoints = true;
//		g_drawSprings = true;
		g_drawEllipsoids = true;
		g_drawMesh = false;
		g_warmup = false;

		return getState();
	}


	/**
	 * initConfig contains configuration for the bar in each instance
	 * 0, 1: x, z position of the bar
	 * 2   : rotation around y axis
	 * 3, 4: x, z linear velocity
	 * 5,  : angular velocity around y axis
	 * 6,7,8: Dimension of the bar
	 */
	void setControllerInit(Eigen::MatrixXd initConfig) {
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();
//		cout<<initConfig<<endl;
		for (int i = 0; i < centers.size(); i++) {
			Eigen::VectorXd config = initConfig.row(i);

			Vec3 center = centers[i];
			Vec3 currPos = center + Vec3(config[0], 0, config[1]);
			Quat currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + config[2]);
			Vec3 currVel = Vec3(config[3], 0, config[4]);
			float currAngVel = config[5];
			currPoses.push_back(currPos);
			currRots.push_back(currRot);
			currVels.push_back(currVel);
			currAngVels.push_back(currAngVel);
//			barDim = Vec3(1.5, 1, 0.01);
			barDim = Vec3(config[6], config[7], config[8]);

		}
	}

	void setGoal(Eigen::MatrixXd goalConfig) {
		goalPos= goalConfig;
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;
		ClearShapes();

		for (int i = 0; i < centers.size(); i++) {

			Vec3 targetPos = centers[i]
					+ Vec3(action(i * actionDim), 0, action(i * actionDim + 1));

			Vec2 targetRotVec = Vec2(action(i * actionDim + 2),
					action(i * actionDim + 3));

//			Vec3 targetPos = centers[i]
//					+ Vec3(0, 0, cosf(g_frame / (60.0 * EIGEN_PI)));
//
//			Vec2 targetRotVec = Vec2(1, 0);

			bool ghost = action(i * actionDim + 4) > 0;

			//				Vec2 targetRotVec = Vec2(1,0);

			int channel = eNvFlexPhaseShapeChannel0;

			if (ghost) {
				channel = channel << 1;
			}
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

			Vec3 newPos = Vec3(currPoses[i].x, currPoses[i].y, currPoses[i].z)
					+ currVels[i] * g_dt;

			Quat newRot = currRots[i]
					* QuatFromAxisAngle(Vec3(0, 1, 0), currAngVels[i] * g_dt);

			currPoses[i] = newPos;
			currRots[i] = newRot;

			for (int t = 5; t < actionDim; t += 2) {
				Vec2 goal_target = Vec2(action(i * actionDim + t),
						action(i * actionDim + t + 1))
						+ Vec2(centers[i][0], centers[i][2]);
				AddSphere(0.12, Vec3(goal_target.x, 0, goal_target.y), Quat(),
						eNvFlexPhaseShapeChannel0 << 1);

			}

			AddBox(Vec3(4, 0.01, 4), centers[i] + Vec3(0, 0.005, 0), Quat(),
					false, eNvFlexPhaseShapeChannel0 << 1);

			AddBox(barDim, newPos, newRot, false, channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1), centers[i] + Vec3(-4, 2, -4), Quat(),
						false, eNvFlexPhaseShapeChannel0 << 1);
			}
		}

		UpdateShapes();


//		if (g_frame % 100==0) {
//			cout << g_frame << endl;
//		}
		return getState();
	}
	void setSceneSeed(int seed) {
		this->seed = seed;
	}
	Eigen::MatrixXd getState() {
		using namespace Eigen;
		int numPart = g_buffers->positions.size();

		int numBars = numSceneDim * numSceneDim;
		//The last four rows are the translational and rotational position and velocity for the moving bar
		MatrixXd state(numPart + 4 * numBars, 3);
		//		cout << state.rows() << endl;
		state.setZero();

		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
			float currCosHalfAng = currRots[i].w;
			float currSinHalfAng = currRots[i].y;
			float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
			float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

			int numPartInScene = numPart / numBars;

			for (int j = 0; j < numPartInScene; j++) {
				state.row(i * (numPartInScene + 4) + j + 4) = Vector3d(
						g_buffers->positions[i * numPartInScene + j].x,
						g_buffers->positions[i * numPartInScene + j].y,
						g_buffers->positions[i * numPartInScene + j].z)
						- Vector3d(cent.x, cent.y, cent.z);
			}
			state.row(i * (numPartInScene + 4)) = Vector3d(currPoses[i].x,
					currPoses[i].y, currPoses[i].z)
					- Vector3d(cent.x, cent.y, cent.z);

			state.row(i * (numPartInScene + 4) + 1) = Vector3d(currCosAng, 0,
					currSinAng);
			state.row(i * (numPartInScene + 4) + 2) = Vector3d(currVels[i].x,
					currVels[i].y, currVels[0].z);
			state.row(i * (numPartInScene + 4) + 3) = Vector3d(
					cos(currAngVels[0]), 0, sin(currAngVels[0]));

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


	virtual void CenterCamera() {
		Vec3 scenelower, sceneupper;

		GetParticleBounds(scenelower, sceneupper);

		g_camPos = Vec3((scenelower.x + sceneupper.x) * 0.5f, 10.0f,
				sceneupper.z );
		g_camAngle = Vec3(0, -DegToRad(85.0f), 0.0f);
	}
};
