#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
//

class GranularSweep: public Scene {
public:

	GranularSweep(const char* name) :
			Scene(name) {
	}
	Vec3 currPos;
	Quat currRot;
	Vec2 currRotVec;

	Vec3 currVel;
	float currAngVel;

	float kp_pos = 0.5;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;

	int seed = -1;
	virtual Eigen::MatrixXd Initialize(int placeholder=0) {

		if(seed!=-1){
			srand(seed);
		}
		// granular pile
		float radius = 0.075f;
		float interRadius = radius * 1.01f;

//		for(int i=-2;i<3;i++){
//			for(int j=0;j<2;j++){
//				Vec3 center = Vec3(i*10,0,j*10);
//				CreateParticleGrid(center+Vec3(0, 3, 0), 50, 1, 50, interRadius, Vec3(0, 0, 0),
//								1, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide),
//								0.003f);
//			}
//
//		}
		Vec3 center = Vec3(0, 0, 0);
		CreateParticleGrid(center + Vec3(0, 1, 0), 50, 1, 50, interRadius,
				Vec3(0, 0, 0), 1, false, 0.0f,
				NvFlexMakePhase(0, eNvFlexPhaseSelfCollide), 0.001f);

		Eigen::Vector2d randPos;
		randPos.setRandom();
		randPos = randPos * 0.01;
		Eigen::VectorXd randRot(1);
		randRot.setRandom();
		randRot *= EIGEN_PI * 0.1;
		currPos = Vec3(0.0f, 0.0f, 2.5f) + Vec3(randPos[0], 0, randPos[1]);
		currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));

		AddBox(Vec3(0.7, 0.5, 0.03), currPos, currRot);

		float currCosHalfAng = currRot.w;
		float currSinHalfAng = currRot.y;

		float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
		float currSinAng = 2 * currCosHalfAng * currSinHalfAng;

		currRotVec = Vec2(currCosAng, currSinAng);

		g_numSubsteps = 2;
		g_params.radius = radius;
		g_params.staticFriction = 1.0f;
		g_params.dynamicFriction = 0.8f;
		g_params.viscosity = 0.0f;
		g_params.numIterations = 10;
		g_params.particleCollisionMargin = g_params.radius * 0.25f;	// 5% collision margin
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
		g_colors[0] = Colour(0.805f, 0.702f, 0.401f);

		currVel = Vec3(0, 0, 0);
		currAngVel = 0;

		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;

		//		action = VectorXd(4);
		//		action << 0, 0, 0, 1;
//		cout << "Num of Particles: " << g_buffers->positions[0] << endl;
//
//		cout << "Angle Diff: " << angDiff << endl;
//		cout << "Position Diff: " << posDiff.x << " " << posDiff.y << " "
//				<< posDiff.z << endl;

		if (g_frame > 40) {
			Vec3 targetPos = Vec3(action(0), 0, action(1));
			Vec2 targetRotVec = Vec2(action(2), action(3));

			float angDiff = VectorToAngle(targetRotVec)
					- VectorToAngle(currRotVec);

			if (angDiff > EIGEN_PI) {
				angDiff -= 2 * EIGEN_PI;
			} else if (angDiff <= -EIGEN_PI) {
				angDiff += 2 * EIGEN_PI;
			}

			Vec3 posDiff = targetPos - currPos;

			Vec3 force = posDiff * kp_pos - currVel * kd_pos;
			float torque = angDiff * kp_rot - currAngVel * kd_rot;

			currVel += force * g_dt;
			currAngVel += torque * g_dt;

			Vec3 newPos = Vec3(currPos.x, currPos.y, currPos.z)
					+ currVel * g_dt;
			Quat newRot = currRot
					* QuatFromAxisAngle(Vec3(0, 1, 0), currAngVel * g_dt);
			currPos = newPos;
			currRot = newRot;
			float currCosHalfAng = currRot.w;
			float currSinHalfAng = currRot.y;
			float currCosAng = Sqr(currCosHalfAng) - Sqr(currSinHalfAng);
			float currSinAng = 2 * currCosHalfAng * currSinHalfAng;
			currRotVec = Vec2(currCosAng, currSinAng);

			ClearShapes();
			AddBox(Vec3(0.7, 0.5, 0.03), newPos, newRot);
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
		//The last four rows are the translational and rotational position and velocity for the moving bar
		MatrixXd state(numPart + 4, 2);
		state.setZero();
//		cout<<g_buffers->positions[0].x<<endl;
//		state.row(0) = Vector2d(g_buffers->positions[0].x,
//							g_buffers->positions[0].z);
		for (int i = 0; i < numPart; i++) {
			state.row(i) = Vector2d(g_buffers->positions[i].x,
					g_buffers->positions[i].z);
//			state.row(i) = Vector2d(1,4);
		}
//		cout << "2" << endl;

		state.row(numPart) = Vector2d(currPos.x, currPos.z);
		state.row(numPart + 1) = Vector2d(currRotVec.x, currRotVec.y);
		state.row(numPart + 2) = Vector2d(currVel.x, currVel.z);
		state.row(numPart + 3) = Vector2d(cos(currAngVel), sin(currAngVel));
//		cout << "THERETHERE" << endl;
		return state;

	}
};
