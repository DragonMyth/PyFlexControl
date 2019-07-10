#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
//

class GranularSweepThreeBars: public Scene {
public:

	GranularSweepThreeBars(const char* name) :
			Scene(name) {
	}

	vector<Vec3> currPosesBar1;
	vector<Vec3> currPosesBar2;
	vector<Vec3> currVelsBar1;
	vector<Vec3> currVelsBar2;

	vector<Quat> currRotsBase;
	vector<float> currAngVelsBase;
	float kp_pos = 0.5;
	float kd_pos = 1.2;
	float maxVel = 1.0f;
	float maxAngVel = EIGEN_PI / 12;

	int particleDim = 30;
	int numSceneDim = 5;
	int seed = -1;
	vector<Vec3> centers;
	Vec3 barHalfDim1 = Vec3(0.01, 0.5, 0.7);
	Vec3 barHalfDim2 = Vec3(0.7, 0.5, 0.01);

	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {
		centers.clear();
		currPosesBar1.clear();
		currPosesBar2.clear();
		currVelsBar1.clear();
		currVelsBar2.clear();
		currRotsBase.clear();
		currAngVelsBase.clear();

		if (seed != -1) {
			srand(seed);

		}
		// granular pile
		float radius = 0.05f;

		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {

				int group = i * numSceneDim + j;

				Vec3 center = Vec3(i * 30, 0, j * 30);
				centers.push_back(center);
				int channel = eNvFlexPhaseShapeChannel0;
				int phase = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide, channel);

				Vec2 xRange = Vec2(-2, 2);
				Vec2 zRange = Vec2(-2, 2);
				Vec2 gap = Vec2(particleDim, particleDim);

				Eigen::VectorXf randRot(1);
				randRot.setRandom();
				randRot *= EIGEN_PI;
				Quat baseRot;
				baseRot = QuatFromAxisAngle(Vec3(0, -1, 0), randRot(0));

				CreateGranularGrid(center, xRange, zRange, radius, gap,
						Vec3(0, 0, 0), 1, false, 0.0f, phase, 0.0f,
						Vec2(cosf(randRot[0]), sinf(randRot[0])));

				Eigen::Vector2d randPos1;
				randPos1.setRandom();
				randPos1 *= 3;

				Eigen::Vector2d randPos2;
				randPos2.setRandom();
				randPos2 *= 3;

				Vec3 currPos1;
				Vec3 currPos2;

				Vec3 currVel1;
				Vec3 currVel2;

				float currAngVel;

				currPos1 = center + Vec3(randPos1[0], 0, randPos1[1]);
				currPos2 = center + Vec3(randPos2[0], 0, randPos2[1]);

				currVel1 = Vec3(0, 0, 0);
				currVel2 = Vec3(0, 0, 0);

				currAngVel = 0;

				currPosesBar1.push_back(currPos1);
				currPosesBar2.push_back(currPos2);
				currVelsBar1.push_back(currVel1);
				currVelsBar2.push_back(currVel2);
				currRotsBase.push_back(baseRot);
				currAngVelsBase.push_back(currAngVel);

				group++;

			}

		}

		g_numSubsteps = 1;
		g_params.radius = radius;
		g_params.staticFriction = 10.5f;
		g_params.dynamicFriction = 1.2f;
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
		g_drawMesh = false;
		g_warmup = false;

		// hack, change the color of phase 0 particles to 'sand'		
//		g_colors[0] = Colour(0.805f, 0.702f, 0.401f);

		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;
		ClearShapes();
		int numPart = g_buffers->positions.size();
		int numBars = numSceneDim * numSceneDim;
		for (int i = 0; i < centers.size(); i++) {

			Vec3 lrtarg = centers[i]
					+ Vec3(action(i * 7), 0, action(i * 7 + 1));
			Vec3 tdtarg = centers[i]
					+ Vec3(action(i * 7 + 2), 0, action(i * 7 + 3));

			float torque = action(i * 7 + 4);

//			Vec3 lrBarAccel = Vec3(1, 0, 0);
//			Vec3 tdBarAccel = Vec3(0, 0, 1);
//			float torque = 0;
			Vec3 goal_target = Vec3(action(i * 7 + 5), 0, action(i * 7 + 6))
					+ centers[i];
//			Vec3 goal_target = Vec3(1,0,0)+ centers[i];
			Vec3 targetDir1 = goal_target-Vec3(currPosesBar1[i].x,0,currPosesBar1[i].z);
			Vec3 targetDir2 = goal_target-Vec3(currPosesBar2[i].x,0,currPosesBar2[i].z);

			//This makes sure the bar does not affect particles when moving away from goal
			float relativeDir1 = Dot(targetDir1, currVelsBar1[i]);
			float relativeDir2 = Dot(targetDir2, currVelsBar2[i]);

			int channel1 = eNvFlexPhaseShapeChannel0;
			int channel2 = eNvFlexPhaseShapeChannel0;

			if (relativeDir1 < 0) {
				channel1 = channel1 << 1;
			}
			if (relativeDir2 < 0) {
							channel2 = channel2 << 1;
			}

			Vec3 force1 = (lrtarg - currPosesBar1[i]) * kp_pos
					- currVelsBar1[i] * kd_pos;
			Vec3 force2 = (tdtarg - currPosesBar2[i]) * kp_pos
					- currVelsBar2[i] * kd_pos;

			currVelsBar1[i] += force1 * g_dt;

			currVelsBar2[i] += force2 * g_dt;

			currAngVelsBase[i] += torque * g_dt;

			currVelsBar1[i].x = minf(maxf(currVelsBar1[i].x, -maxVel), maxVel);
			currVelsBar1[i].z = minf(maxf(currVelsBar1[i].z, -maxVel), maxVel);

			currVelsBar2[i].x = minf(maxf(currVelsBar2[i].x, -maxVel), maxVel);
			currVelsBar2[i].z = minf(maxf(currVelsBar2[i].z, -maxVel), maxVel);

			currAngVelsBase[i] = minf(maxf(currAngVelsBase[i], -maxAngVel),
					maxAngVel);

			Vec3 newPos1 = currPosesBar1[i] + currVelsBar1[i] * g_dt;
			Vec3 newPos2 = currPosesBar2[i] + currVelsBar2[i] * g_dt;

			newPos1.x = maxf(minf(newPos1.x - centers[i].x, 4), -4)
					+ centers[i].x;
			newPos1.y = maxf(minf(newPos1.y - centers[i].y, 4), -4)
					+ centers[i].y;
			newPos1.z = maxf(minf(newPos1.z - centers[i].z, 4), -4)
					+ centers[i].z;

			newPos2.x = maxf(minf(newPos2.x - centers[i].x, 4), -4)
					+ centers[i].x;
			newPos2.y = maxf(minf(newPos2.y - centers[i].y, 4), -4)
					+ centers[i].y;
			newPos2.z = maxf(minf(newPos2.z - centers[i].z, 4), -4)
					+ centers[i].z;

			Quat newRot = currRotsBase[i]
					* QuatFromAxisAngle(Vec3(0, -1, 0),
							currAngVelsBase[i] * g_dt);

			int numPartInScene = numPart / numBars;

			for (int j = 0; j < numPartInScene; j++) {
				float cos = cosf(currAngVelsBase[i] * g_dt);
				float sin = sinf(currAngVelsBase[i] * g_dt);

				Vec4 position = g_buffers->positions[i * numPartInScene + j];
				position.x -= centers[i].x;
				position.z -= centers[i].z;

				float newX = position.x * cos - position.z * sin;
				float newZ = position.x * sin + position.z * cos;
				position.x = newX;
				position.z = newZ;
				position.x += centers[i].x;
				position.z += centers[i].z;
				g_buffers->positions[i * numPartInScene + j].x = position.x;
				g_buffers->positions[i * numPartInScene + j].z = position.z;

			}
			currPosesBar1[i] = newPos1;
			currPosesBar2[i] = newPos2;

			currRotsBase[i] = newRot;

			AddSphere(0.12, Vec3(goal_target.x, 0, goal_target.z), Quat(),
					eNvFlexPhaseShapeChannel0 << 1);

			AddBox(Vec3(4, 0.004, 4), centers[i] + Vec3(0, 0.005, 0),
					currRotsBase[i], false, eNvFlexPhaseShapeChannel0);

			AddBox(barHalfDim1, newPos1, Quat(), false, channel1);
			AddBox(barHalfDim2, newPos2, Quat(), false, channel2);

		}

		UpdateShapes();

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
		MatrixXd state(numPart + 6 * numBars, 2);
		state.setZero();

		for (int i = 0; i < numBars; i++) {
			Vec3 cent = centers[i];
			float currCosHalfAng = currRotsBase[i].w;
			float currSinHalfAng = currRotsBase[i].y;
			float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
			float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

			int numPartInScene = numPart / numBars;

			for (int j = 0; j < numPartInScene; j++) {
				state.row(i * (numPartInScene + 6) + j + 6) = Vector2d(
						g_buffers->positions[i * numPartInScene + j].x,
						g_buffers->positions[i * numPartInScene + j].z)
						- Vector2d(cent.x, cent.z);
			}

			state.row(i * (numPartInScene + 6)) = Vector2d(currPosesBar1[i].x,
					currPosesBar1[i].z) - Vector2d(cent.x, cent.z);
			state.row(i * (numPartInScene + 6) + 1) = Vector2d(
					currPosesBar2[i].x, currPosesBar2[i].z)
					- Vector2d(cent.x, cent.z);

			state.row(i * (numPartInScene + 6) + 2) = Vector2d(currCosAng,
					currSinAng);

			state.row(i * (numPartInScene + 6) + 3) = Vector2d(
					currVelsBar1[i].x, currVelsBar1[0].z);
			state.row(i * (numPartInScene + 6) + 4) = Vector2d(
					currVelsBar2[i].x, currVelsBar2[0].z);
			state.row(i * (numPartInScene + 6) + 5) = Vector2d(
					currAngVelsBase[0], currAngVelsBase[0]);
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
