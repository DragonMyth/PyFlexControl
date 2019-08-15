#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
//

class GranularSweepControllableGhostManualControl: public Scene {
public:

	GranularSweepControllableGhostManualControl(const char* name) :
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
	int particleDim = 32;
	int numSceneDim = 1;
	int seed = -1;
	vector<Vec3> centers;
	Vec3 barDim = Vec3(0.7, 0.5, 0.01);
	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {
		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		float radius = 0.05f;

//		int group =0;
		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {

				Vec3 center = Vec3(i * 15, 0, j * 15);
				centers.push_back(center);
				int group = centers.size();
				int channel = eNvFlexPhaseShapeChannel0;
				int phase = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide, channel);

				Vec2 xRange = Vec2(-2, 2);
				Vec2 zRange = Vec2(-2, 2);
				Vec2 gap = Vec2(particleDim, particleDim);

				CreateGranularGrid(center, xRange, zRange, radius, gap,
						Vec3(0, 0, 0), 1, false, 0.0f, phase, 0.0f);
				//Random sample a initial position in range [-2,2] x [-2,2].

				Eigen::Vector2f randInitPos;
				randInitPos.setRandom();
				randInitPos *= 2;

				Eigen::VectorXf randInitShape(1);
				randInitShape.setRandom();
				randInitShape /= 2.0f;

//				if(randInitShape(0)<0){
//					CreateParticleGridRand(center+Vec3(randInitPos(0),radius,randInitPos(1)),particleDim/2,4,particleDim/2,radius*3,Vec3(0,0,0),1,false,0.0f,phase,0.005f);
//				}else{
//					CreateGranularGrid(center,xRange,zRange,radius,gap,Vec3(0,0,0),1,false,0.0f,phase,0.0f);
//
//				}

				Vec3 currPos;
				Quat currRot;

				Vec3 currVel;
				float currAngVel;

				Eigen::VectorXf initAngOnCirc(1);
				initAngOnCirc.setRandom();
				initAngOnCirc *= EIGEN_PI;
//				float initAngOnCirc = 0;

				Eigen::Vector2d randPos(cosf(initAngOnCirc(0)) * 3,
						sinf(initAngOnCirc(0)) * 3);

//				Eigen::Vector2d randPos;
//				randPos.setRandom();
//				randPos = randPos*1;

				Eigen::VectorXf randRot(1);
				randRot.setRandom();
				randRot *= EIGEN_PI;

				currPos = center + Vec3(randPos[0], 0, randPos[1]);

//				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), EIGEN_PI/2-initAngOnCirc);
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
		g_lightDistance *= 100.5f;

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
		ClearShapes();

		for (int i = 0; i < centers.size(); i++) {

			Vec3 targetPos = centers[i]
					+ Vec3(action(i * 7), 0, action(i * 7 + 1));

			Vec2 targetRotVec = Vec2(action(i * 7 + 2), action(i * 7 + 3));

			bool ghost = action(i * 7 + 4) > 0 ? true : false;

//				Vec2 targetRotVec = Vec2(1,0);

			Vec2 goal_target = Vec2(action(i * 7 + 5), action(i * 7 + 6))
					+ Vec2(centers[i][0], centers[i][2]);

//				Vec2 goal_target = Vec2(0,0)+Vec2(centers[i][0],centers[i][2]);
			Vec2 targetDir = goal_target - Vec2(currPoses[i].x, currPoses[i].z);

//			float relativeDir = Dot(targetDir,
//					Vec2(currVels[i].x, currVels[i].z));

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

			AddBox(Vec3(4, 0.004, 4), centers[i] + Vec3(0, 0.005, 0), Quat(),
					false, eNvFlexPhaseShapeChannel0 << 1);

			AddBox(barDim, newPos, newRot, false, channel);

			if (ghost) {
				AddBox(Vec3(1, 1, 1), centers[i] + Vec3(-4, 2, -4), Quat(),
						false, eNvFlexPhaseShapeChannel0 << 1);
			}

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
		MatrixXd state(numPart + 4 * numBars, 3);

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
};
