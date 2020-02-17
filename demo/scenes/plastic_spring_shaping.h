#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
using namespace std;

//

class PlasticSpringShaping: public Scene {
public:

	// General Param for the simulation
	int numSceneDim = 7;
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

	float kp_pos = 0.3;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	Vec3 barDim = Vec3(1.5, 1, 0.01);

	// Array of hash maps that store the neighbourhood of particles for which springs will be added.
	map<int, std::vector<int>> *springFuseMap;

	// Array of hash maps that store existing spring pairs between particles
	map<std::pair<int, int>, int> *bidirSpringMap;

	// Stores the number of spings connected to each particle. Used for limiting the max spring connection
	Eigen::VectorXi perPartSpringCnt;
	float stiffness = 1.5f;

	float springFuseDist = radius * 2.0f;
	float springBreakDist = radius * 2.5f;
	float springCompressThreshold = radius * 1.9f;
	float springStrechThreshold = radius * 2.3f;
	float minSpringDist = radius*1.3;

	int maxSpringPerPart = 27;
	PlasticSpringShaping(const char* name) :
			Scene(name) {

		goalPos = Eigen::MatrixXd(numSceneDim * numSceneDim, 2);
		goalPos.setZero();
		partInitialization = Eigen::MatrixXd(numSceneDim * numSceneDim, 6);
		partInitialization.setZero();
		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			partInitialization(i, 3) = 5;
			partInitialization(i, 4) = 5;
			partInitialization(i, 5) = 5;
		}

	}

	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {

		springFuseMap = new map<int, std::vector<int>> [numSceneDim
				* numSceneDim];
		bidirSpringMap = new map<std::pair<int, int>, int> [numSceneDim
				* numSceneDim];
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
							particleClusterParam(cluster + 1),
							particleClusterParam(cluster + 2));
					int clusterDimx = (int) (particleClusterParam(cluster + 3));
					int clusterDimy = (int) (particleClusterParam(cluster + 4));
					int clusterDimz = (int) (particleClusterParam(cluster + 5));
//
					CreateSpringCubeAroundCenter(center + offsetPos,
							clusterDimx, clusterDimy, clusterDimz,
							springFuseDist/sqrt(2), phase1, stiffness, stiffness,
							stiffness, 0.0f, 1.0f);
//					CreateGranularCubeAroundCenter(center + offsetPos,
//												clusterDimx, clusterDimy, clusterDimz,
//												radius * 1.7f, phase1, Vec3(0.0, 0.0, 0.0), 1.0f,0.0f);
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
				barDim = Vec3(0.7, 0.5, 0.01);

				//Random sample a initial position in range [-2,2] x [-2,2].

			}

		}
		cout << "Number of Particles Per instance: " << numPartPerScene << endl;

		perPartSpringCnt = Eigen::VectorXi(g_buffers->positions.size());
		perPartSpringCnt.setZero();
		updateSpaceMap();

		Eigen::VectorXd tempAct(numSceneDim * numSceneDim * actionDim);
		tempAct.setZero();
		updateSprings(tempAct);

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

	/**
	 * Update the unified space partition for the neightbourhood of particles to fuse spring
	 */
	void updateSpaceMap() {

		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			springFuseMap[i].clear();
		}

		int fuseGridSize = ceil((playgroundHalfExtent * 2.0) / springFuseDist);

		for (int i = 0; i < g_buffers->positions.size(); i++) {
			Vec3 idxVecFuseMap = getMapIdx(i, fuseGridSize);

			int idxFuseMap = idxVecFuseMap.z * fuseGridSize * fuseGridSize
					+ idxVecFuseMap.y * fuseGridSize + idxVecFuseMap.x;

			int group = i / numPartPerScene;

			if (springFuseMap[group].count(idxFuseMap) == 0) {
				std::vector<int> list;
				list.resize(0);
				springFuseMap[group].insert(
						std::pair<int, std::vector<int>>(idxFuseMap, list));
			}

			springFuseMap[group][idxFuseMap].push_back(i);

		}

	}

	/*
	 * Get Map index of a particle
	 */
	Vec3 getMapIdx(int partIdx, int gridSize) {
		int group = partIdx / (numPartPerScene);
		Vec3 center = centers[group];
		Vec3 pos = Vec3(g_buffers->positions[partIdx]) - center
				+ Vec3(playgroundHalfExtent, 0, playgroundHalfExtent);
		Vec3 mapIdx = Vec3(
				int(pos.x / (playgroundHalfExtent * 2.0f) * gridSize),
				int(pos.y / (playgroundHalfExtent * 2.0f) * gridSize),
				int(pos.z / (playgroundHalfExtent * 2.0f) * gridSize));
		return mapIdx;
	}

	bool cutParticles(Vec3 p, Vec3 q, int group, int ghost) {
		if (ghost > 0) {
			return false;
		}

		Vec3 barPose = currPoses[group];
		Vec3 barRot = currRots[group];
		Quat quat = QuatFromAxisAngle(Vec3(0, 1, 0), barRot.y)
				* QuatFromAxisAngle(Vec3(1, 0, 0), barRot.x);

		Vec3 bar_u = Rotate(quat, Vec3(1, 0, 0));
		Vec3 bar_v = Rotate(quat, Vec3(0, 1, 0));

		Vec3 uxv = Cross(bar_u, bar_v);

		// Check if particles are on opposite side of bar
		float val1 = Dot(uxv, Normalize(p - barPose));
		float val2 = Dot(uxv, Normalize(q - barPose));

		// If two particles are on same side, they are NOT cut by the bar. Else check if in range of the bar
		if (val1 * val2 >= 0) {
			return false;
		} else {
			Vec3 barCenter = barPose + barDim[1] * bar_v;
			float t = -Dot((p - barCenter), uxv) / Dot(q - p, uxv);
			Vec3 inter = p + t * (q - p);
			float horProj = Dot((inter - barCenter), bar_u);
			float vertProj = Dot((inter - barCenter), bar_v);

			if (Abs(horProj) <= barDim[0] && Abs(vertProj) <= barDim[1]) {
				return true;
			} else {
				return false;
			}
			return true;
		}

	}

	/*
	 * Fake the plastic behavior
	 * Set the spring rest length beyond certain threshold
	 */
	float calNewSpringRestLength(float length, float currRestLength) {
		float res = currRestLength;
		if (length <= springCompressThreshold) {
			res = fmax(length, minSpringDist);
//			res = length;
		} else if (length >= springStrechThreshold) {
			res = length;
		}
		return res;
	}

	/*
	 * Update the spring connectivity between particles
	 */

	void updateSprings(Eigen::VectorXd action) {

		int fuseGridSize = ceil((playgroundHalfExtent * 2) / springFuseDist);

		std::vector<int> mConstraintIndices;
		std::vector<float> mConstraintCoefficients;
		std::vector<float> mConstraintRestLengths;
		mConstraintIndices.resize(0);
		mConstraintCoefficients.resize(0);
		mConstraintRestLengths.resize(0);
		perPartSpringCnt.setZero();

		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			bidirSpringMap[i].clear();
		}

		// Keep all springs that are within break distance or not cut by the bar
		for (int i = 0; i < g_buffers->springLengths.size(); i++) {

			//Get the pair of particles of the spring
			int a = g_buffers->springIndices[i * 2];
			int b = g_buffers->springIndices[i * 2 + 1];

			Vec3 p = Vec3(g_buffers->positions[a]);
			Vec3 q = Vec3(g_buffers->positions[b]);

			int group = a / (numPartPerScene);

			float length = Length(p - q);
			if (length <= springBreakDist
					&& !cutParticles(p, q, group,
							action(group * actionDim + 6))) {
				mConstraintIndices.push_back(a);
				mConstraintIndices.push_back(b);
				mConstraintCoefficients.push_back(stiffness);

				mConstraintRestLengths.push_back(
						calNewSpringRestLength(length,
								g_buffers->springLengths[i]));

				std::pair<int, int> ab = std::pair<int, int>(a, b);
				std::pair<int, int> ba = std::pair<int, int>(b, a);

				bidirSpringMap[group][ab] = 1;
				bidirSpringMap[group][ba] = 1;
				perPartSpringCnt[a]++;
				perPartSpringCnt[b]++;

			}
		}

		//For all particles
		for (int i = 0; i < g_buffers->positions.size(); i++) {
			int group = i / (numPartPerScene);
			Vec3 idxVecFuseMap = getMapIdx(i, fuseGridSize);

//			int remainning_spring = maxSpringPerPart - perPartSpringCnt[i];
			//Look at surrounding 27 neighbours
			for (int x = -1; x <= 1; x++) {
				for (int y = -1; y <= 1; y++) {
					for (int z = -1; z <= 1; z++) {
						Vec3 neighbourIdx = idxVecFuseMap + Vec3(x, y, z);
						//If neighbour index is not out of bound
						if (neighbourIdx.x >= 0 && neighbourIdx.x < fuseGridSize
								&& neighbourIdx.y >= 0
								&& neighbourIdx.y < fuseGridSize
								&& neighbourIdx.z >= 0
								&& neighbourIdx.z < fuseGridSize) {
							int idx = neighbourIdx.z * fuseGridSize
									* fuseGridSize
									+ neighbourIdx.y * fuseGridSize
									+ neighbourIdx.x;
							//If cell contains particles is not empty
							if (springFuseMap[group].count(idx) > 0) {
								// For all particles in the cell, form a spring between particles i and j
								for (int k = 0;
										k < springFuseMap[group][idx].size(); k++) {

									int j = springFuseMap[group][idx][k];


//									std::cout<<i<<"Particle: "<<j<<" Remaining Springs: "<<maxSpringPerPart - perPartSpringCnt[i]<<std::endl;

									if (i != j && maxSpringPerPart - perPartSpringCnt[i] > 0 && maxSpringPerPart - perPartSpringCnt[j] > 0) {
										Vec3 p = Vec3(g_buffers->positions[i]);
										Vec3 q = Vec3(g_buffers->positions[j]);
										float length = Length(p - q);
										std::pair<int, int> ij = std::pair<int,
												int>(i, j);
										std::pair<int, int> ji = std::pair<int,
												int>(j, i);

										//If spring pair is not already created and not cut by the bar, form new spring
										if (bidirSpringMap[group].count(ij) == 0
												&& length <= springFuseDist
												&& !cutParticles(p, q, group,
														action(
																group
																		* actionDim
																		+ 6))) {

											bidirSpringMap[group][ij] = 1;
											bidirSpringMap[group][ji] = 1;

											mConstraintIndices.push_back(i);
											mConstraintIndices.push_back(j);
											mConstraintCoefficients.push_back(
													stiffness);
											mConstraintRestLengths.push_back(
													Length(p - q));

											perPartSpringCnt[i]++;
											perPartSpringCnt[j]++;

										}
									}
								}

							}
						}
					}
				}
			}
		}

		int numSprings = mConstraintCoefficients.size();
		g_buffers->springIndices.assign(&mConstraintIndices[0], numSprings * 2);
		g_buffers->springStiffness.assign(&mConstraintCoefficients[0],
				numSprings);
		g_buffers->springLengths.assign(&mConstraintRestLengths[0], numSprings);

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

			newPos.x = minf(
					maxf(newPos.x - centers[i].x, -playgroundHalfExtent),
					playgroundHalfExtent) + centers[i].x;
			newPos.y = minf(maxf(newPos.y - centers[i].y, 0), 3) + centers[i].y;
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
					centers[i], Quat(), false, eNvFlexPhaseShapeChannel0 << 1);

			Quat quat = QuatFromAxisAngle(Vec3(0, 1, 0), currRots[i].y)
					* QuatFromAxisAngle(Vec3(1, 0, 0), currRots[i].x);

			//Translating the point of rotation to the base of the bar
			Vec3 rotatedVec = Rotate(quat, Vec3(0, 1, 0));

			AddBox(barDim, newPos + barDim[1] * rotatedVec, quat, false,
					channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1),
						centers[i]
								+ Vec3(-playgroundHalfExtent, 2,
										-playgroundHalfExtent), Quat(), false,
						eNvFlexPhaseShapeChannel0 << 1);
			}

		}

		UpdateShapes();

		if (g_frame % 10 == 0) {
			updateSpaceMap();
			updateSprings(action);
		}

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
		g_camPos = Vec3(21.2443f, 44.0126f, 24.5113f);
		g_camAngle = Vec3(0, -1.38404, 0);
	}
};
