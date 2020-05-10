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
	int actionDim = 7;
	float playgroundHalfExtent = 4;

	int numPartPerScene = 0;
	vector<Vec3> centers;
	Eigen::MatrixXd goalPos;

	// m x n matrix, m: number of instances, n: 6k, k is the number of sub-instances, 3 numbers for position offsets, 3 numbers for dimension
	Eigen::MatrixXd partInitialization;

	// Parameters for the controlling bar
	vector<Vec3> currPoses;
	vector<Vec3> currRots; //XYZ order of euler angles
	vector<Vec3> currVels;
	vector<Vec3> currAngVels;

//	float kp_pos = 1.0f;
//	float kd_pos = 2.4f;
//	float kp_rot = 1.0f;
//	float kd_rot = 2.1;
	float kp_pos = 0.3;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	Vec3 barDim = Vec3(0.7, 1.0, 0.01);

	GranularSweepShapingManualControl(const char* name) :
			Scene(name) {

		goalPos = Eigen::MatrixXd(numSceneDim * numSceneDim, 2);
		goalPos.setZero();
		partInitialization = Eigen::MatrixXd(numSceneDim * numSceneDim, 6);
		partInitialization.setZero();


//		allMeshId.resize(0);

		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			partInitialization(i, 3) = 5;
			partInitialization(i, 4) = 5;
			partInitialization(i, 5) = 5;
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
		int group = 0;
		for (int i = 0; i < numSceneDim; i++) {
			for (int j = 0; j < numSceneDim; j++) {


				int idx = i * numSceneDim + j;
//				allMeshId[idx] = (CreateTriangleMesh(mBarMesh));


				Eigen::VectorXd particleClusterParam = partInitialization.row(
						idx);

				Vec3 center = Vec3(i * 15, 0, j * 15);
//				int group = centers.size();

				int phase1 = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
						channel);

				for (int cluster = 0; cluster < particleClusterParam.size();
						cluster += 6) {

					Vec3 offsetPos = Vec3(particleClusterParam(cluster),
							particleClusterParam(cluster + 1),
							particleClusterParam(cluster + 2));
					int clusterDimx = (int) (particleClusterParam(cluster + 3));
					int clusterDimy = (int) (particleClusterParam(cluster + 4));
					int clusterDimz = (int) (particleClusterParam(cluster + 5));
//

					CreateGranularCubeAroundCenter(center + offsetPos,
												clusterDimx, clusterDimy, clusterDimz,
												radius * 1.3f, phase1, Vec3(0.0, 0.0, 0.0), 1.0f,0.05f);
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

				Vec3 currRotEuler = Vec3(0, 0, 0);
//				std::cout << currRotEuler.x << " , " << currRotEuler.y << " , "
//						<< currRotEuler.z << std::endl;
				Vec3 currVel = Vec3(0, 0, 0);
				Vec3 currAngVel = Vec3(0, 0, 0);

				currPoses.push_back(currPos);
				currRots.push_back(currRotEuler);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);
//				barDim = Vec3(1.5, 1, 0.01);
//				barDim = Vec3(0.7, 0.5, 0.01);

				//Random sample a initial position in range [-2,2] x [-2,2].

			}

		}
		cout << "Number of Particles Per instance: " << numPartPerScene << endl;


		g_numSubsteps = 3;

		g_params.radius = radius;
		g_params.staticFriction = 1.8f;
		g_params.dynamicFriction = 1.3f;

		g_params.viscosity = 0.0f;
		g_params.numIterations = 7;
		g_params.sleepThreshold = g_params.radius * 0.25f;
//		g_params.collisionDistance = radius*0.5f;
//		g_params.relaxationMode = eNvFlexRelaxationGlobal;
		g_params.relaxationFactor = 1.0f;
		g_params.damping = 0.24f;
		g_params.particleCollisionMargin = g_params.radius * 0.1f;
		g_params.shapeCollisionMargin = g_params.radius * 0.1f;
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

		for (int i = 0; i < centers.size(); i++) {
			Eigen::VectorXd config = initConfig.row(i);

			Vec3 center = centers[i];
			Vec3 currPos = center + Vec3(config[0], config[1], config[2]);
			Vec3 currEulerRot = Vec3(config[3], config[4], config[5]);
			Vec3 currVel = Vec3(config[6], config[7], config[8]);
			Vec3 currAngVel = Vec3(config[9], config[10], config[11]);
			currPoses.push_back(currPos);
			currRots.push_back(currEulerRot);
			currVels.push_back(currVel);
			currAngVels.push_back(currAngVel);
//			barDim = Vec3(1.5, 1, 0.01);
			barDim = Vec3(config[12], config[13], config[14]);

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
					+ Vec3(action(i * actionDim), action(i * actionDim + 1),
							action(i * actionDim + 2));
//
//			targetPos.x = minf(
//					maxf(targetPos.x - centers[i].x, -playgroundHalfExtent),
//					playgroundHalfExtent) + centers[i].x;
//			targetPos.y = minf(maxf(targetPos.y - centers[i].y, 0), 3) + centers[i].y;
//			targetPos.z = minf(
//					maxf(targetPos.z - centers[i].z, -playgroundHalfExtent),
//					playgroundHalfExtent) + centers[i].z;

			Vec3 targetRotVec = Vec3(action(i * actionDim + 3),
					action(i * actionDim + 4), action(i * actionDim + 5));

			bool ghost = action(i * actionDim + 6) > 0;

			int channel = eNvFlexPhaseShapeChannel0;

			if (ghost) {
				channel = channel << 1;
			}

			Vec3 angDiff = targetRotVec - currRots[i];

			for (int a = 0; a < 3; a++) {
				if (angDiff[a] > EIGEN_PI) {
					angDiff[a] -= 2 * EIGEN_PI;
				} else if (angDiff[a] <= -EIGEN_PI) {
					angDiff[a] += 2 * EIGEN_PI;
				}
			}

			Vec3 posDiff = targetPos - currPoses[i];

			Vec3 force = posDiff * kp_pos - currVels[i] * kd_pos;

			Vec3 torque = angDiff * kp_rot - currAngVels[i] * kd_rot;

			currVels[i] += force * g_dt;

			currAngVels[i] += torque * g_dt;

			Vec3 newPos = Vec3(currPoses[i].x, currPoses[i].y, currPoses[i].z)
					+ currVels[i] * g_dt;

			Vec3 newRot = currRots[i] + currAngVels[i] * g_dt;
			for (int a = 0; a < 3; a++) {
				if (newRot[a] > EIGEN_PI) {
					newRot[a] -= 2 * EIGEN_PI;
				} else if (newRot[a] <= -EIGEN_PI) {
					newRot[a] += 2 * EIGEN_PI;
				}
			}

			newRot[0] = minf(maxf(newRot[0], -EIGEN_PI / 2),
			EIGEN_PI / 2);


			if(newPos.x-centers[i].x<-playgroundHalfExtent || newPos.x-centers[i].x>playgroundHalfExtent ){
				currVels[i].x = 0;
			}
			if(newPos.z-centers[i].z<-playgroundHalfExtent || newPos.z-centers[i].z>playgroundHalfExtent ){
				currVels[i].z = 0;
			}
			if(newPos.y-centers[i].y<0 || newPos.y-centers[i].y>3 ){
				currVels[i].y = 0;
			}

			newPos.x = minf(
					maxf(newPos.x - centers[i].x, -playgroundHalfExtent),
					playgroundHalfExtent) + centers[i].x;
			newPos.y = minf(maxf(newPos.y - centers[i].y, 0), 3) + centers[i].y;
			newPos.z = minf(
					maxf(newPos.z - centers[i].z, -playgroundHalfExtent),
					playgroundHalfExtent) + centers[i].z;





			Vec3 oldPos = currPoses[i];
			Vec3 oldRot = currRots[i];
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
					centers[i], Quat(), false, eNvFlexPhaseShapeChannel0 << 1);

			Quat quat = QuatFromAxisAngle(Vec3(0, 1, 0), currRots[i].y)
					* QuatFromAxisAngle(Vec3(1, 0, 0), currRots[i].x);

			//Translating the point of rotation to the base of the bar
			Vec3 rotatedVec = Rotate(quat, Vec3(0, 1, 0));

			Quat oldQuat = QuatFromAxisAngle(Vec3(0, 1, 0), oldRot.y)
					* QuatFromAxisAngle(Vec3(1, 0, 0), oldRot.x);
			Vec3 oldRotatedVec = Rotate(oldQuat, Vec3(0, 1, 0));


			AddBox(barDim,newPos + barDim[1] * rotatedVec, quat,false,channel,Vec3(0.3f,0.3f,1.0f));
//			AddTriangleMesh(allMeshId[i+numSceneDim*numSceneDim], newPos + barDim[1] * rotatedVec, quat, Vec3(1.0f),
//					Vec3(0.3, 0.3, 1.0));

			g_buffers->shapePrevPositions[g_buffers->shapePrevPositions.size()
					- 1] = Vec4(oldPos + barDim[1] * oldRotatedVec, 0.0f);
			g_buffers->shapePrevRotations[g_buffers->shapePrevPositions.size()
					- 1] = oldQuat;


			float linearVelThresh = 0.9f;
//			float angVelThresh = 0.5f;
//			if (!(abs(currVels[i].x) > linearVelThresh || abs(currVels[i].y) > linearVelThresh
//					|| abs(currVels[i].z) > linearVelThresh || abs(currAngVels[i].x) > angVelThresh
//					|| abs(currAngVels[i].y) > angVelThresh
//					|| abs(currAngVels[i].z) > angVelThresh)) {
//				g_buffers->shapePrevPositions[g_buffers->shapePrevPositions.size()
//						- 1] = Vec4(oldPos + barDim[1] * oldRotatedVec, 0.0f);
//				g_buffers->shapePrevRotations[g_buffers->shapePrevPositions.size()
//						- 1] = oldQuat;
//			}
			if (Length(currVels[i]) > linearVelThresh) {


//				float t = maxf(1-(Length(currVels[i])-linearVelThresh)/linearVelThresh,0);
				float t = 0.6;

				Vec3 interpPos = (oldPos * t + newPos * (1 - t));
				g_buffers->shapePrevPositions[g_buffers->shapePrevPositions.size()
						- 1] = Vec4(interpPos + barDim[1] * oldRotatedVec,
						0.0f);
			}

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
			if (abs(g_buffers->positions[k].y) < 0.11) {
				g_buffers->phases[k] = phase2;
//				cout<<g_buffers->positions[k].y<<endl;
			} else {

				g_buffers->phases[k] = phase1;
			}
		}

//		cout<<"Current Velocity: "<<currVels[0].x<<" "<<currVels[0].y<<" "<<currVels[0].z<<" "<<endl;
//		if (g_frame % 100==0) {
//			cout << g_frame << endl;
//		}
		//		cout<<"Cam X: "<<g_camPos.x<<"Cam Y: "<<g_camPos.y<<"Cam Z: "<<g_camPos.z<<"Cam Angle X: "<<g_camAngle.x<<"Cam Angle Y: "<<g_camAngle.y<<"Cam Angle Z: "<<g_camAngle.z<<endl;

		return getState();
	}

	virtual void Sync() {

		// update solver data not already updated in the main loop
		NvFlexSetSprings(g_solver, g_buffers->springIndices.buffer,
				g_buffers->springLengths.buffer,
				g_buffers->springStiffness.buffer,
				g_buffers->springLengths.size());
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

		state.setZero();
		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
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

			state.row(i * (numPartInScene + 4) + 1) = Vector3d(currRots[i].x,
					currRots[i].y, currRots[i].z);

			state.row(i * (numPartInScene + 4) + 2) = Vector3d(currVels[i].x,
					currVels[i].y, currVels[i].z);

			state.row(i * (numPartInScene + 4) + 3) = Vector3d(currAngVels[i].x,
					currAngVels[i].y, currAngVels[i].z);

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

