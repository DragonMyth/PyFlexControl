
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
using namespace std;

//


class GranularSweepShapingManualControl: public Scene {
public:

	// General Param for the simulation
	int numSceneDim = 1;
	int seed = -1;
//	int dimx = 10;
//	int dimy = 2;
//	int dimz = 10;
	float radius = 0.2f;
	int actionDim = 6;
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


	GranularSweepShapingManualControl(const char* name) :
			Scene(name) {

		goalPos = Eigen::MatrixXd(numSceneDim * numSceneDim, 2);
		goalPos.setZero();
		partInitialization = Eigen::MatrixXd(numSceneDim * numSceneDim, 6);
		partInitialization.setZero();
		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			partInitialization(i, 3) = 6;
			partInitialization(i, 4) = 6;
			partInitialization(i, 5) = 6;
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
//				int group = centers.size();

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
							particleClusterParam(cluster+1), particleClusterParam(cluster+2));
					int clusterDimx = (int)(particleClusterParam(cluster+3));
					int clusterDimy = (int)(particleClusterParam(cluster+4));
					int clusterDimz = (int)(particleClusterParam(cluster+5));
//
//					CreateSpringCubeAroundCenter(center + offsetPos, clusterDimx,
//							clusterDimy, clusterDimz, springRestLength, phase1, stiffness,
//							stiffness, stiffness, 0.0f, 1.0f);
					CreateGranularCubeAroundCenter(center + offsetPos,
												clusterDimx, clusterDimy, clusterDimz,
												radius * 1.7f, phase1, Vec3(0.0, 0.0, 0.0), 1.0f,0.0f);
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
		cout << numPartPerScene << endl;

		g_numSubsteps = 1;
		g_params.radius = radius;
//		g_params.staticFriction = 10.5f;
		g_params.staticFriction = 0.2f;

		g_params.dynamicFriction = 1.2f;
//		g_params.dynamicFriction = 0.2f;

		g_params.numIterations = 4;
		g_params.particleCollisionMargin = g_params.radius * 0.05f;	// 5% collision margin
		g_params.sleepThreshold = g_params.radius * 0.25f;
		g_params.shockPropagation = 6.f;
		g_params.restitution = 0.2f;
		g_params.relaxationFactor = 1.f;
		g_params.damping = 0.14f;
		g_params.numPlanes = 1;

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
			Vec3 currPos = center + Vec3(config[0], config[1], config[2]);
			Quat currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + config[3]);
			Vec3 currVel = Vec3(config[4], config[5], config[6]);
			float currAngVel = config[7];
			currPoses.push_back(currPos);
			currRots.push_back(currRot);
			currVels.push_back(currVel);
			currAngVels.push_back(currAngVel);
//			barDim = Vec3(1.5, 1, 0.01);
			barDim = Vec3(config[8], config[9], config[10]);

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
					+ Vec3(action(i * actionDim), action(i * actionDim+1), action(i * actionDim + 2));

			Vec2 targetRotVec = Vec2(action(i * actionDim + 3),
					action(i * actionDim + 4));

//			Vec3 targetPos = centers[i]
//					+ Vec3(0, 0, cosf(g_frame / (60.0 * EIGEN_PI)));
//
//			Vec2 targetRotVec = Vec2(1, 0);

			bool ghost = action(i * actionDim + 5) > 0;

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
			newPos.y = minf(
								maxf(newPos.y - centers[i].y, 0),
								3) + centers[i].y;
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

			AddBox(Vec3(playgroundHalfExtent, 0.01, playgroundHalfExtent), centers[i] + Vec3(0, 0.005, 0), Quat(),
					false, eNvFlexPhaseShapeChannel0 << 1);

			AddBox(barDim, newPos+Vec3(0,barDim[1],0), newRot, false, channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1),
						centers[i]
								+ Vec3(-playgroundHalfExtent, 2,
										-playgroundHalfExtent), Quat(), false,
						eNvFlexPhaseShapeChannel0 << 1);
			}
		}

		UpdateShapes();

		int phase2 = NvFlexMakePhaseWithChannels(1,
								eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
								eNvFlexPhaseShapeChannel0);
		int phase1 = NvFlexMakePhaseWithChannels(0,
										eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
										eNvFlexPhaseShapeChannel0);
		for (int k = 0; k < g_buffers->positions.size(); k++) {
			if (abs(g_buffers->positions[k].y)< 0.11) {
				g_buffers->phases[k] = phase2;
//				cout<<g_buffers->positions[k].y<<endl;
			}else{

				g_buffers->phases[k] = phase1;
			}
		}
//		if (g_frame % 10 == 0) {
//			updateSpaceMap();
//			updateSprings(action);
//		}

//		if (g_frame % 100==0) {
//			cout << g_frame << endl;
//		}
		//		cout<<"Cam X: "<<g_camPos.x<<"Cam Y: "<<g_camPos.y<<"Cam Z: "<<g_camPos.z<<"Cam Angle X: "<<g_camAngle.x<<"Cam Angle Y: "<<g_camAngle.y<<"Cam Angle Z: "<<g_camAngle.z<<endl;

		return getState();
	}

	virtual void Sync() {

		// update solver data not already updated in the main loop
//		NvFlexSetSprings(g_solver, g_buffers->springIndices.buffer,
//				g_buffers->springLengths.buffer,
//				g_buffers->springStiffness.buffer,
//				g_buffers->springLengths.size());
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

		g_camPos = Vec3((scenelower.x + sceneupper.x) * 0.5f, 10.0f,
				sceneupper.z);
		g_camAngle = Vec3(0, -DegToRad(81.0f), 0.0f);
	}
};


