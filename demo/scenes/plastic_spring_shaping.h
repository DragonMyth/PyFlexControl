#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
using namespace std;

//

class PlasticSpringShaping: public Scene {
public:

	PlasticSpringShaping(const char* name) :
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
	int numSceneDim = 4;
	int seed = -1;
	vector<Vec3> centers;
	Vec3 barDim = Vec3(1.5, 1, 0.01);
	float maxDeformRate = 0.1f;
	float springBreakRatio = 0.6f;

	float springRestLength;
	map<int, std::vector<int>> *springFuseMap;
	map<int, std::vector<int>> *springBreakMap;

	map<std::pair<int, int>, int> *bidirSpringMap;
	Eigen::VectorXi perPartSpringCnt;

	int dimx = 12;
	int dimy = 2;
	int dimz = 12;

	float stiffness = 1.0f;

	float radius = 0.15f;

	float springFuseDist = radius * 1.5f;
	float springBreakDist = radius * 4.0f;

	float springCompressThreshold = radius * 2.25f;
	float springStrechThreshold = radius * 3.0f;

	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {

		springFuseMap = new map<int, std::vector<int>> [numSceneDim
				* numSceneDim];
		springBreakMap = new map<int, std::vector<int>> [numSceneDim
				* numSceneDim];

		bidirSpringMap = new map<std::pair<int, int>, int> [numSceneDim
				* numSceneDim];
		g_lightDistance *= 100.5f;

		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		Vec3 lower, upper;
		springRestLength = radius * 2.0f;
		for (int i = 0; i < numSceneDim; i++) {
			for (int j = 0; j < numSceneDim; j++) {

				Vec3 center = Vec3(i * 15, 0, j * 15);
				int group = centers.size();
				int channel = eNvFlexPhaseShapeChannel0;

				int phase = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter,
						channel);

				CreateSpringCube(center + Vec3(0, 0, 0), dimx, dimy, dimz,
						springRestLength, phase, stiffness, stiffness,
						stiffness, 0.0f, 1.0f);

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

				//Random sample a initial position in range [-2,2] x [-2,2].

				Eigen::Vector2f randInitPos;
				randInitPos.setRandom();
				randInitPos *= 2;

				Eigen::VectorXf randInitShape(1);
				randInitShape.setRandom();
				randInitShape /= 2.0f;

				Vec3 currPos;
				Quat currRot;

				Vec3 currVel;
				float currAngVel;

				Eigen::VectorXf initAngOnCirc(1);
				initAngOnCirc.setRandom();
				initAngOnCirc *= EIGEN_PI;
//				initAngOnCirc = EIGEN_PI;

				Eigen::Vector2d randPos(cosf(initAngOnCirc(0)) * 3,
						sinf(initAngOnCirc(0)) * 3);

//				Eigen::Vector2d randPos;
//				randPos.setRandom();
//				randPos = randPos*1;

				Eigen::VectorXf randRot(1);
				randRot.setRandom();
				randRot *= EIGEN_PI;
//				randRot *= 0;

				currPos = center + Vec3(randPos[0], 0, randPos[1]);
//				currPos = center + Vec3(0, 0, 0);
				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));

				currVel = Vec3(0, 0, 0);
				currAngVel = 0;

				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);

				group++;

			}

		}
		perPartSpringCnt = Eigen::VectorXi(g_buffers->positions.size());
		perPartSpringCnt.setZero();
		updateSpaceMap();
		updateSprings();

		g_params.radius = radius;
		g_params.dynamicFriction = 7.55f;
		g_params.staticFriction = 50.5f;
		g_params.dissipation = 0.0f;
		g_params.numIterations = 4;
		g_params.viscosity = 0.0f;
		g_params.drag = 0.05f;
		g_params.collisionDistance = radius;
		g_params.shapeCollisionMargin = radius * 0.1;
		g_params.relaxationFactor = 0.5f;
		g_windStrength = 0.0f;

		g_numSubsteps = 3;

		// draw options
		g_drawPoints = true;
//		g_drawSprings = true;
		g_drawMesh = false;
		g_warmup = false;

		return getState();
	}

	void updateSpaceMap() {
		for (int i = 0; i < numSceneDim * numSceneDim; i++) {
			springFuseMap[i].clear();
		}

		int fuseGridSize = ceil(8.0 / springFuseDist);

		for (int i = 0; i < g_buffers->positions.size(); i++) {
			Vec3 idxVecFuseMap = getMapIdx(i, fuseGridSize);

			int idxFuseMap = idxVecFuseMap.z * fuseGridSize * fuseGridSize
					+ idxVecFuseMap.y * fuseGridSize + idxVecFuseMap.x;

			int group = i / (dimx * dimy * dimz);

			if (springFuseMap[group].count(idxFuseMap) == 0) {
				std::vector<int> list;
				list.resize(0);
				springFuseMap[group].insert(
						std::pair<int, std::vector<int>>(idxFuseMap, list));
			}

			springFuseMap[group][idxFuseMap].push_back(i);

		}

	}

	Vec3 getMapIdx(int partIdx, int gridSize) {
		int group = partIdx / (dimx * dimy * dimz);
		Vec3 center = centers[group];
		Vec3 pos = Vec3(g_buffers->positions[partIdx]) - center + Vec3(4, 0, 4);
		Vec3 mapIdx = Vec3(int(pos.x / 8.0 * gridSize),
				int(pos.y / 8.0 * gridSize), int(pos.z / 8.0 * gridSize));
		return mapIdx;

	}

	bool cutParticles(Vec3 p, Vec3 q, int group) {
		Vec3 barPose = currPoses[group];
		float currCosHalfAng = currRots[group].w;
		float currSinHalfAng = currRots[group].y;

		float barRotCos = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
		float barRotSin = -2 * currCosHalfAng * currSinHalfAng;

		Vec3 endPoint1Pos = barPose + barDim.x * Vec3(barRotCos, 0, barRotSin);
		Vec3 endPoint2Pos = barPose - barDim.x * Vec3(barRotCos, 0, barRotSin);

		float a1 = endPoint1Pos.z - endPoint2Pos.z;
		float b1 = endPoint2Pos.x - endPoint1Pos.x;

		float c1 = a1 * endPoint2Pos.x + b1 * endPoint2Pos.z;

		float a2 = p.z - q.z;
		float b2 = q.x - p.x;
		float c2 = a2 * q.x + b2 * q.z;

		float det = a1 * b2 - a2 * b1;
		float x = (c1 * b2 - c2 * b1) / det;
		float z = (a1 * c2 - a2 * c1) / det;

		if (det == 0) {
			return false;
		}

		if (minf(endPoint2Pos.x, endPoint1Pos.x) <= x
				&& maxf(endPoint2Pos.x, endPoint1Pos.x) >= x
				&& minf(p.x, q.x) <= x && maxf(p.x, q.x) >= x

				&& minf(endPoint2Pos.z, endPoint1Pos.z) <= z
				&& maxf(endPoint2Pos.z, endPoint1Pos.z) >= z
				&& minf(p.z, q.z) <= z && maxf(p.z, q.z) >= z) {

			return true;
		} else {
			return false;
		}

	}

	float calNewSpringRestLength(float length, float currRestLength) {
		float res = currRestLength;
		if (length <= springCompressThreshold) {
			res = length;
		} else if (length >= springStrechThreshold) {
			res = length;
		}
		return res;
	}

	void updateSprings() {
		int fuseGridSize = ceil(8.0 / springFuseDist);

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

		for (int i = 0; i < g_buffers->springLengths.size(); i++) {
			int a = g_buffers->springIndices[i * 2];
			int b = g_buffers->springIndices[i * 2 + 1];

			Vec3 p = Vec3(g_buffers->positions[a]);
			Vec3 q = Vec3(g_buffers->positions[b]);
			int group = a / (dimx * dimy * dimz);

			float length = Length(p - q);
			if (length <= springBreakDist && !cutParticles(p, q, group)) {
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
			int group = i / (dimx * dimy * dimz);
			Vec3 idxVecFuseMap = getMapIdx(i, fuseGridSize);

			int remainning_spring = 25 - perPartSpringCnt[i];
			//Look at surrounding 27 neighbours
			for (int x = -1; x <= 1; x++) {
				for (int y = -1; y <= 1; y++) {
					for (int z = -1; z <= 1; z++) {
						Vec3 neighbourIdx = idxVecFuseMap + Vec3(x, y, z);
						//If neighbour index is out of bound
						if (neighbourIdx.x >= 0 && neighbourIdx.x < fuseGridSize
								&& neighbourIdx.y >= 0
								&& neighbourIdx.y < fuseGridSize
								&& neighbourIdx.z >= 0
								&& neighbourIdx.z < fuseGridSize) {
							int idx = neighbourIdx.z * fuseGridSize
									* fuseGridSize
									+ neighbourIdx.y * fuseGridSize
									+ neighbourIdx.x;
							//If neighbour is not empty
							if (springFuseMap[group].count(idx) > 0) {

								for (int k = 0;
										k < springFuseMap[group][idx].size()
												&& remainning_spring > 0; k++) {
									int j = springFuseMap[group][idx][k];
									if (i != j) {
										Vec3 p = Vec3(g_buffers->positions[i]);
										Vec3 q = Vec3(g_buffers->positions[j]);
										float length = Length(p - q);
										std::pair<int, int> ij = std::pair<int,
												int>(i, j);
										std::pair<int, int> ji = std::pair<int,
												int>(j, i);
										//If spring pair is not already created
										if (bidirSpringMap[group].count(ij) == 0
												&& length <= springFuseDist
												&& !cutParticles(p, q, group)) {
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

		int actionDim = 7;
		for (int i = 0; i < centers.size(); i++) {

			Vec3 targetPos = centers[i]
					+ Vec3(action(i * actionDim), 0, action(i * actionDim + 1));

			Vec2 targetRotVec = Vec2(action(i * actionDim + 2),
					action(i * actionDim + 3));

//			Vec3 targetPos = centers[i]
//					+ Vec3(0, 0, cosf(g_frame / (60.0 * EIGEN_PI)));
//
//			Vec2 targetRotVec = Vec2(1, 0);

			bool ghost = action(i * actionDim + 4) > 0 ? true : false;

			//				Vec2 targetRotVec = Vec2(1,0);

			Vec2 goal_target = Vec2(action(i * actionDim + 5),
					action(i * actionDim + 6))
					+ Vec2(centers[i][0], centers[i][2]);

			Vec2 targetDir = goal_target - Vec2(currPoses[i].x, currPoses[i].z);

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

			AddSphere(0.12, Vec3(goal_target.x, 0, goal_target.y), Quat(),
					eNvFlexPhaseShapeChannel0 << 1);

			AddBox(Vec3(4, 0.01, 4), centers[i] + Vec3(0, 0.005, 0), Quat(),
					false, eNvFlexPhaseShapeChannel0 << 1);

			AddBox(barDim, newPos, newRot, false, channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1), centers[i] + Vec3(-4, 2, -4), Quat(),
						false, eNvFlexPhaseShapeChannel0 << 1);
			}
		}

		UpdateShapes();

		if (g_frame % 15 == 0) {
			updateSpaceMap();
			updateSprings();
		}

//		if (g_frame % 100==0) {
//			cout << g_frame << endl;
//		}
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

		g_camPos = Vec3((scenelower.x + sceneupper.x) * 0.5f, 30.0f,
				(scenelower.z + sceneupper.z) * 0.5f);
		g_camAngle = Vec3(0, -DegToRad(85.0f), 0.0f);
	}
};
