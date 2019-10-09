#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
using namespace std;

//

class GranularSweepShaping: public Scene {
public:

	// General Param for the simulation
	int numSceneDim = 7;
	int seed = -1;
//	int dimx = 10;
//	int dimy = 2;
//	int dimz = 10;
	float radius = 0.2f;
	int actionDim = 5;
	float playgroundHalfExtent = 4;

	int numPartPerScene = 0;
	vector<Vec3> centers;
	Eigen::MatrixXd goalPos;

	// m x n matrix, m: number of instances, n: 6k, k is the number of sub-instances, 3 numbers for position offsets, 3 numbers for dimension
	Eigen::MatrixXd partInitialization;

	// Parameters for the controlling bar
	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;

	float kp_pos = 0.3;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	Vec3 barDim = Vec3(1.5, 1, 0.01);



	GranularSweepShaping(const char* name) :
			Scene(name) {

		goalPos = Eigen::MatrixXd(numSceneDim * numSceneDim, 2);
		goalPos.setZero();
		partInitialization = Eigen::MatrixXd(numSceneDim * numSceneDim, 6);
		partInitialization.setZero();
		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			partInitialization(i, 3) = 10;
			partInitialization(i, 4) = 2;
			partInitialization(i, 5) = 10;
		}

	}

	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {

		g_lightDistance *= 100.5f;
		centers.resize(0);

		currPoses.resize(0);
		currRots.resize(0);
		currVels.resize(0);
		currAngVels.resize(0);
		centers.clear();

		Vec3 lower, upper;
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();
		int channel = eNvFlexPhaseShapeChannel0;
		int group =0;

		for (int i = 0; i < numSceneDim; i++) {
			for (int j = 0; j < numSceneDim; j++) {
				int idx = i * numSceneDim + j;
				Eigen::VectorXd particleClusterParam = partInitialization.row(
						idx);

				Vec3 center = Vec3(i * 15, 0, j * 15);

				int phase1 = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
						channel);

				int phase2 = NvFlexMakePhaseWithChannels(group + 1,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
						channel);

				int offset = g_buffers->positions.size();
				for (int cluster = 0; cluster < particleClusterParam.size();
						cluster += 6) {

					Vec3 offsetPos = Vec3(particleClusterParam(cluster),
							particleClusterParam(cluster + 1),
							particleClusterParam(cluster + 2));
					int clusterDimx = (int) (particleClusterParam(cluster + 3));
					int clusterDimy = (int) (particleClusterParam(cluster + 4));
					int clusterDimz = (int) (particleClusterParam(cluster + 5));

					CreateGranularCubeAroundCenter(center + offsetPos,
							clusterDimx, clusterDimy, clusterDimz,
							radius * 1.7f, phase1, Vec3(0.0,0.0,0.0), 1.0f);
				}
				if (i == 0 && j == 0) {
					numPartPerScene = g_buffers->positions.size();
				}

				centers.push_back(center);

//				for (int k = offset; k < (offset + numPartPerScene); k++) {
//					if (g_buffers->positions[k].z - center[2] > 0) {
//						g_buffers->phases[k] = phase2;
//					}
//				}

				Vec3 currPos = center + Vec3(0, 0, 0);
				Quat currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0);
				Vec3 currVel = Vec3(0, 0, 0);
				float currAngVel = 0;
				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);
				barDim = Vec3(1.5, 1, 0.01);

				//Random sample a initial position in range [-2,2] x [-2,2].

			}

		}


		Eigen::VectorXd tempAct(numSceneDim * numSceneDim * actionDim);
		tempAct.setZero();
		cout << numPartPerScene << endl;

		g_params.radius = radius;
//		g_params.fluidRestDistance = radius;

		g_params.dynamicFriction = 7.55f;
		g_params.staticFriction = 30.5f;
		g_params.dissipation = 0.0f;
		g_params.numIterations = 2;
		g_params.viscosity = 0.0f;
		g_params.drag = 0.05f;
		g_params.collisionDistance = radius * 0.5f;
		g_params.shapeCollisionMargin = radius * 0.1;
		g_params.relaxationFactor = 0.5f;
		g_windStrength = 0.0f;

		g_numSubsteps = 2;

		// draw options
		g_drawPoints = true;
//		g_drawSprings = true;
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
		currPoses.resize(0);
		currRots.resize(0);
		currVels.resize(0);
		currAngVels.resize(0);

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
		goalPos = goalConfig;
	}

	void setInitClusterParam(Eigen::MatrixXd initClusterParam) {
		partInitialization = initClusterParam;
	}

	void setMapHalfExtent(float mapHalfExtent) {
		playgroundHalfExtent = mapHalfExtent;
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

//			bool ghost = action(i * actionDim + 4) > 0;

			bool ghost = (action(i * actionDim + 4) > 0);
			//				Vec2 targetRotVec = Vec2(1,0);

			int channel = eNvFlexPhaseShapeChannel0;
//			cout<< action(i * actionDim + 4)<<endl;

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

			newPos.x = minf(
					maxf(newPos.x - centers[i].x, -playgroundHalfExtent),
					playgroundHalfExtent) + centers[i].x;
			newPos.z = minf(
					maxf(newPos.z - centers[i].z, -playgroundHalfExtent),
					playgroundHalfExtent) + centers[i].z;

			currPoses[i] = newPos;
			currRots[i] = newRot;

			Eigen::VectorXd goals = goalPos.row(i);
			for (int t = 0; t < goals.size(); t += 2) {
				Vec2 goal_target = Vec2(goals[t], goals[t + 1])
						+ Vec2(centers[i][0], centers[i][2]);
				AddSphere(0.3, Vec3(goal_target.x, 0, goal_target.y), Quat(),
						eNvFlexPhaseShapeChannel0 << 1);

			}

			AddBox(Vec3(playgroundHalfExtent, 0.01, playgroundHalfExtent),
					centers[i] + Vec3(0, 0.005, 0), Quat(), false,
					eNvFlexPhaseShapeChannel0 << 1);

			AddBox(barDim, newPos, newRot, false, channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1),
						centers[i]
								+ Vec3(-playgroundHalfExtent, 2,
										-playgroundHalfExtent), Quat(), false,
						eNvFlexPhaseShapeChannel0 << 1);
			}
		}

		UpdateShapes();

//		cout<<"Cam X: "<<g_camPos.x<<"Cam Y: "<<g_camPos.y<<"Cam Z: "<<g_camPos.z<<"Cam Angle X: "<<g_camAngle.x<<"Cam Angle Y: "<<g_camAngle.y<<"Cam Angle Z: "<<g_camAngle.z<<endl;
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
//				cout<<"ASGF"<<g_buffers->positions[i * numPartInScene + j].x<<endl;

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

		/*g_camPos = Vec3((scenelower.x + sceneupper.x) * 0.5f, 20.0f,
				(scenelower.z + sceneupper.z) * 0.5f);
		g_camAngle = Vec3(0, -DegToRad(85.0f), 0.0f);*/

		// 4x4
//		g_camPos = Vec3(21.7816f,63.1574f,27.1928f);
//		g_camAngle = Vec3(0,-1.5132,0);
		// 7x7
//		g_camPos = Vec3(35.1552f,118.898f,33.0568);
//		g_camAngle = Vec3(0,-1.65806,0);

		// 3x4
		g_camPos = Vec3(21.2443f,44.0126f,24.5113f);
		g_camAngle = Vec3(0,-1.38404,0);
	}
};
