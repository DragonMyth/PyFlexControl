#include <iostream>
#include <eigen3/Eigen/Dense>
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
	int numSceneDim = 3;
	int seed = -1;
	vector<Vec3> centers;
	Vec3 barDim = Vec3(1.5, 1, 0.01);
	float maxDeformRate = 0.3f;
	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {
		g_lightDistance *= 100.5f;

		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		float radius = 0.15f;

		float stretchStiffness = 1.0f;
		float bendStiffness = .8f;
		float shearStiffness = 0.5f;

		int dimx = 16;
		int dimy = 4;

		int dimz = 16;
		Vec3 lower, upper;

		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {

				Vec3 center = Vec3(i * 15, 0, j * 15);
				int group = centers.size();
				int channel = eNvFlexPhaseShapeChannel0;

				int phase = NvFlexMakePhaseWithChannels(group,
						eNvFlexPhaseSelfCollide, channel);

				CreateSpringCube(center + Vec3(0, 0, 0), dimx, dimy, dimz,
						radius, phase, stretchStiffness, bendStiffness,
						shearStiffness, 0.0f, 1.0f);

				if (i == -numSceneDim / 2 && j == -numSceneDim / 2) {
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

//		g_numSubsteps = 1;
//		g_params.radius = radius;
//		g_params.staticFriction = 10.5f;
//		g_params.dynamicFriction = 1.2f;
//		g_params.viscosity = 0.0f;
//		g_params.numIterations = 3;
//		g_params.particleCollisionMargin = g_params.radius * 0.05f;	// 5% collision margin
//		g_params.sleepThreshold = g_params.radius * 0.25f;
//		g_params.shockPropagation = 6.f;
//		g_params.restitution = 0.2f;
//		g_params.relaxationFactor = 1.f;
//		g_params.damping = 0.14f;
//		g_params.numPlanes = 1;

		g_params.radius = radius;
		g_params.dynamicFriction = 0.1625f;
		g_params.staticFriction = 10.5f;
		g_params.dissipation = 0.0f;
		g_params.numIterations = 4;
		g_params.viscosity = 0.0f;
		g_params.drag = 0.05f;
		g_params.collisionDistance = radius;
		g_params.shapeCollisionMargin = radius * 0.5f;
		g_params.relaxationFactor = 1.3f;
		g_windStrength = 0.0f;

		g_numSubsteps = 1;

		// draw options
		g_drawPoints = true;
		g_drawSprings = true;
		g_drawMesh = false;
		g_warmup = false;

		// hack, change the color of phase 0 particles to 'sand'		
//		g_colors[0] = Colour(0.805f, 0.702f, 0.401f);

		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;
		ClearShapes();

		int actionDim = 7;
		for (int i = 0; i < centers.size(); i++) {
//
//			Vec3 targetPos = centers[i]
//					+ Vec3(action(i * actionDim), 0, action(i * actionDim + 1));


			Vec3 targetPos = centers[i]
					+ Vec3(0, 0, cos(g_frame / 60.0f / (EIGEN_PI)));

			Vec2 targetRotVec = Vec2(action(i * actionDim + 2),
					action(i * actionDim + 3));



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

		std::vector<int> mConstraintIndices;
		std::vector<float> mConstraintCoefficients;
		std::vector<float> mConstraintRestLengths;
		mConstraintIndices.resize(0);
		mConstraintCoefficients.resize(0);
		mConstraintCoefficients.resize(0);
		for (int i = 0; i < g_buffers->springLengths.size(); i++) {
			int a = g_buffers->springIndices[i * 2];
			int b = g_buffers->springIndices[i * 2 + 1];

			Vec3 p = Vec3(g_buffers->positions[a]);
			Vec3 q = Vec3(g_buffers->positions[b]);
			float deformRatio = (g_buffers->springLengths[i] - Length(p - q))
					/ g_buffers->springLengths[i];
			if (deformRatio >= -maxDeformRate) {

				if (deformRatio > maxDeformRate) {
					g_buffers->springLengths[i] = Length(p - q);

				}
				mConstraintIndices.push_back(a);
				mConstraintIndices.push_back(b);
				mConstraintCoefficients.push_back(
						g_buffers->springStiffness[i]);
				mConstraintRestLengths.push_back(g_buffers->springLengths[i]);
			}

		}
		int numSprings = mConstraintCoefficients.size();
		g_buffers->springIndices.assign(&mConstraintIndices[0], numSprings * 2);
		g_buffers->springStiffness.assign(&mConstraintCoefficients[0],
				numSprings);
		g_buffers->springLengths.assign(&mConstraintRestLengths[0], numSprings);

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
};
