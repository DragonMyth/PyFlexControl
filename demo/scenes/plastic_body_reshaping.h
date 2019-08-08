#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;

class PlasticBodyReshaping: public Scene {

public:
	PlasticBodyReshaping(const char* name) :
			Scene(name), mRadius(0.2f), mRelaxationFactor(1.0f), plasticDeformation(
					true) {
		const Vec3 colorPicker[7] = { Vec3(0.0f, 0.5f, 1.0f), Vec3(0.797f,
				0.354f, 0.000f), Vec3(0.000f, 0.349f, 0.173f), Vec3(0.875f,
				0.782f, 0.051f), Vec3(0.01f, 0.170f, 0.453f), Vec3(0.673f,
				0.111f, 0.000f), Vec3(0.612f, 0.194f, 0.394f) };
		memcpy(mColorPicker, colorPicker, sizeof(Vec3) * 7);
	}

	vector<Vec3> currPoses;
	vector<Quat> currRots;
	vector<Vec3> currVels;
	vector<float> currAngVels;

	float kp_pos = 0.5;
	float kd_pos = 1.2;
	float kp_rot = 0.7;
	float kd_rot = 1;
	int numSceneDim = 3;
	int seed = -1;
	vector<Vec3> centers;
	Vec3 barDim = Vec3(0.7, 0.5, 0.01);

	float mRadius;
	float mRelaxationFactor;

	Vec3 mColorPicker[7];

	struct Instance {
		Instance(const char* mesh) :

				mFile(mesh), mColor(0.5f, 0.5f, 1.0f),

				mScale(2.0f), mTranslation(0.0f, 1.0f, 0.0f),

				mClusterSpacing(1.0f), mClusterRadius(0.0f), mClusterStiffness(
						0.5f),

				mLinkRadius(0.0f), mLinkStiffness(1.0f),

				mGlobalStiffness(0.0f),

				mSurfaceSampling(0.0f), mVolumeSampling(4.0f),

				mSkinningFalloff(2.0f), mSkinningMaxDistance(100.0f),

				mClusterPlasticThreshold(0.0f), mClusterPlasticCreep(0.0f) {
		}

		const char* mFile;
		Vec3 mColor;

		Vec3 mScale;
		Vec3 mTranslation;

		float mClusterSpacing;
		float mClusterRadius;
		float mClusterStiffness;

		float mLinkRadius;
		float mLinkStiffness;

		float mGlobalStiffness;

		float mSurfaceSampling;
		float mVolumeSampling;

		float mSkinningFalloff;
		float mSkinningMaxDistance;

		float mClusterPlasticThreshold;
		float mClusterPlasticCreep;
	};

	std::vector<Instance> mInstances;

private:

	struct RenderingInstance {
		Mesh* mMesh;
		std::vector<int> mSkinningIndices;
		std::vector<float> mSkinningWeights;
		vector<Vec3> mRigidRestPoses;
		Vec3 mColor;
		int mOffset;
	};

	std::vector<RenderingInstance> mRenderingInstances;

	bool plasticDeformation;

public:
	virtual Eigen::MatrixXd Initialize(int placeholder = 0) {

		centers.clear();
		currPoses.clear();
		currRots.clear();
		currVels.clear();
		currAngVels.clear();

		PlasticBodyReshaping::Instance plasticThinBox("/home/yzhang/FleX_PyBind11/data/box_high.ply");
		plasticThinBox.mScale = Vec3(17, 3.5f, 17);
		plasticThinBox.mClusterSpacing = 1.5f;
		plasticThinBox.mClusterRadius = 0.0f;
		plasticThinBox.mClusterStiffness = 0.0f;
		plasticThinBox.mGlobalStiffness = 0.7f;
		plasticThinBox.mClusterPlasticThreshold = 1e-5f;
		plasticThinBox.mClusterPlasticCreep = 0.01f;
		plasticThinBox.mTranslation[1] = 0.01f;
		plasticThinBox.mVolumeSampling = 2;
		float radius = mRadius;

		g_solverDesc.featureMode = eNvFlexFeatureModeSimpleSolids;
		g_params.radius = radius;
		g_params.dynamicFriction = 0.35f;
		g_params.particleFriction = 0.25f;
		g_params.numIterations = 4;
		g_params.collisionDistance = radius*1.01f;

		g_params.relaxationFactor = mRelaxationFactor;
		g_lightDistance *= 100.5f;

		g_windStrength = 0.0f;
		g_numSubsteps = 2;

		// draw options
		g_wireframe = false;
		g_drawSprings = false;
		g_drawBases = false;

		g_drawMesh = false;
		g_drawPoints = true;

		g_buffers->rigidOffsets.push_back(0);

		mRenderingInstances.resize(0);
		Vec3 lower, upper;
		for (int i = -numSceneDim / 2; i < numSceneDim / 2 + 1; i++) {
			for (int j = -numSceneDim / 2; j < numSceneDim / 2 + 1; j++) {
				Vec3 center = Vec3(i * 15, 0, j * 15);

				plasticThinBox.mTranslation[0] = center.x;
				plasticThinBox.mTranslation[2] = center.z;

				mInstances.push_back(plasticThinBox);

				// build soft bodies
				CreateSoftBody(plasticThinBox, mRenderingInstances.size());

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

				Vec3 currPos;
				Quat currRot;

				Vec3 currVel;
				float currAngVel;

				Eigen::VectorXf initAngOnCirc(1);
				initAngOnCirc.setRandom();
				initAngOnCirc *= EIGEN_PI;
//				initAngOnCirc(0) = EIGEN_PI / 2;

				Eigen::Vector2d randPos(cosf(initAngOnCirc(0)) * 3,
						sinf(initAngOnCirc(0)) * 3);

				Eigen::VectorXf randRot(1);
				randRot.setRandom();
//				randRot *= 0;

				currPos = center + Vec3(randPos[0], 0, randPos[1]);

				currRot = QuatFromAxisAngle(Vec3(0, 1, 0), 0 + randRot(0));
				currVel = Vec3(0, 0, 0);
				currAngVel = 0;

				currPoses.push_back(currPos);
				currRots.push_back(currRot);
				currVels.push_back(currVel);
				currAngVels.push_back(currAngVel);
			}
		}

		for (int i = 0; i < int(g_buffers->positions.size()); ++i)
			if (g_buffers->positions[i].y < g_params.radius * 0.7f)
				g_buffers->positions[i].w = 0.0f;

		return getState();
	}

	Eigen::MatrixXd Update(Eigen::VectorXd action) {
		using namespace Eigen;
		ClearShapes();

		int actionDim = 7;
		for (int i = 0; i < centers.size(); i++) {

//			Vec3 targetPos = centers[i]
//					+ Vec3(action(i * actionDim), 0, action(i * actionDim + 1));

			Vec3 targetPos = centers[i]
					+ Vec3(0, 0, cos(g_frame / 60.0f / (EIGEN_PI)));

			Vec2 targetRotVec = Vec2(action(i * actionDim + 2),
					action(i * actionDim + 3));
			int channel = eNvFlexPhaseShapeChannel0;

			bool ghost = action(i * actionDim + 4) > 0 ? true : false;
			if (ghost) {
				channel = channel << 1;
			}
			Vec2 goal_target = Vec2(action(i * actionDim + 5),
					action(i * actionDim + 6))
					+ Vec2(centers[i][0], centers[i][2]);

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

			Vec3 newPos = Vec3(currPoses[i].x, barDim.y + (g_params.radius * 0),
					currPoses[i].z) + currVels[i] * g_dt;

			Quat newRot = currRots[i]
					* QuatFromAxisAngle(Vec3(0, 1, 0), currAngVels[i] * g_dt);

			currPoses[i] = newPos;
			currRots[i] = newRot;

			AddSphere(0.12, Vec3(goal_target.x, 0, goal_target.y), Quat(),
					eNvFlexPhaseShapeChannel0 << 1);

			AddBox(Vec3(4, 0.008, 4), centers[i] + Vec3(0, 0.005, 0), Quat(),
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

	void CreateSoftBody(Instance instance, int group = 0) {

		RenderingInstance renderingInstance;

		Mesh* mesh = ImportMesh(GetFilePathByPlatform(instance.mFile).c_str());

		mesh->Normalize();

		mesh->Transform(
				TranslationMatrix(Point3(instance.mTranslation))
						* ScaleMatrix(instance.mScale * mRadius));

		renderingInstance.mMesh = mesh;
		renderingInstance.mColor = instance.mColor;
		renderingInstance.mOffset = g_buffers->rigidTranslations.size();

		double createStart = GetSeconds();

		// create soft body definition
		NvFlexExtAsset* asset = NvFlexExtCreateSoftFromMesh(
				(float*) &renderingInstance.mMesh->m_positions[0],
				renderingInstance.mMesh->m_positions.size(),
				(int*) &renderingInstance.mMesh->m_indices[0],
				renderingInstance.mMesh->m_indices.size(), mRadius,
				instance.mVolumeSampling, instance.mSurfaceSampling,
				instance.mClusterSpacing * mRadius,
				instance.mClusterRadius * mRadius, instance.mClusterStiffness,
				instance.mLinkRadius * mRadius, instance.mLinkStiffness,
				instance.mGlobalStiffness, instance.mClusterPlasticThreshold,
				instance.mClusterPlasticCreep);

		double createEnd = GetSeconds();

		// create skinning
		const int maxWeights = 4;

		renderingInstance.mSkinningIndices.resize(
				renderingInstance.mMesh->m_positions.size() * maxWeights);
		renderingInstance.mSkinningWeights.resize(
				renderingInstance.mMesh->m_positions.size() * maxWeights);

		for (int i = 0; i < asset->numShapes; ++i)
			renderingInstance.mRigidRestPoses.push_back(
					Vec3(&asset->shapeCenters[i * 3]));

		double skinStart = GetSeconds();

		NvFlexExtCreateSoftMeshSkinning(
				(float*) &renderingInstance.mMesh->m_positions[0],
				renderingInstance.mMesh->m_positions.size(),
				asset->shapeCenters, asset->numShapes,
				instance.mSkinningFalloff, instance.mSkinningMaxDistance,
				&renderingInstance.mSkinningWeights[0],
				&renderingInstance.mSkinningIndices[0]);

		double skinEnd = GetSeconds();

		printf("Created soft in %f ms Skinned in %f\n",
				(createEnd - createStart) * 1000.0f,
				(skinEnd - skinStart) * 1000.0f);

		const int particleOffset = g_buffers->positions.size();
		const int indexOffset = g_buffers->rigidOffsets.back();

		// add particle data to solver
		for (int i = 0; i < asset->numParticles; ++i) {
			g_buffers->positions.push_back(&asset->particles[i * 4]);
			g_buffers->velocities.push_back(0.0f);

			const int phase = NvFlexMakePhase(group,
					eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
			g_buffers->phases.push_back(phase);
		}

		// add shape data to solver
		for (int i = 0; i < asset->numShapeIndices; ++i)
			g_buffers->rigidIndices.push_back(
					asset->shapeIndices[i] + particleOffset);

		for (int i = 0; i < asset->numShapes; ++i) {
			g_buffers->rigidOffsets.push_back(
					asset->shapeOffsets[i] + indexOffset);
			g_buffers->rigidTranslations.push_back(
					Vec3(&asset->shapeCenters[i * 3]));
			g_buffers->rigidRotations.push_back(Quat());
			g_buffers->rigidCoefficients.push_back(asset->shapeCoefficients[i]);
		}

		// add plastic deformation data to solver, if at least one asset has non-zero plastic deformation coefficients, leave the according pointers at NULL otherwise
		if (plasticDeformation) {
			if (asset->shapePlasticThresholds && asset->shapePlasticCreeps) {
				for (int i = 0; i < asset->numShapes; ++i) {
					g_buffers->rigidPlasticThresholds.push_back(
							asset->shapePlasticThresholds[i]);
					g_buffers->rigidPlasticCreeps.push_back(
							asset->shapePlasticCreeps[i]);
				}
			} else {
				for (int i = 0; i < asset->numShapes; ++i) {
					g_buffers->rigidPlasticThresholds.push_back(0.0f);
					g_buffers->rigidPlasticCreeps.push_back(0.0f);
				}
			}
		} else {
			if (asset->shapePlasticThresholds && asset->shapePlasticCreeps) {
				int oldBufferSize = g_buffers->rigidCoefficients.size()
						- asset->numShapes;

				g_buffers->rigidPlasticThresholds.resize(oldBufferSize);
				g_buffers->rigidPlasticCreeps.resize(oldBufferSize);

				for (int i = 0; i < oldBufferSize; i++) {
					g_buffers->rigidPlasticThresholds[i] = 0.0f;
					g_buffers->rigidPlasticCreeps[i] = 0.0f;
				}

				for (int i = 0; i < asset->numShapes; ++i) {
					g_buffers->rigidPlasticThresholds.push_back(
							asset->shapePlasticThresholds[i]);
					g_buffers->rigidPlasticCreeps.push_back(
							asset->shapePlasticCreeps[i]);
				}

				plasticDeformation = true;
			}
		}

		// add link data to the solver 
		for (int i = 0; i < asset->numSprings; ++i) {
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 0]);
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 1]);

			g_buffers->springStiffness.push_back(asset->springCoefficients[i]);
			g_buffers->springLengths.push_back(asset->springRestLengths[i]);
		}

		NvFlexExtDestroyAsset(asset);

		mRenderingInstances.push_back(renderingInstance);
	}

	virtual void Draw(int pass) {
		if (!g_drawMesh)
			return;

		for (int s = 0; s < int(mRenderingInstances.size()); ++s) {
			const RenderingInstance& instance = mRenderingInstances[s];

			Mesh m;
			m.m_positions.resize(instance.mMesh->m_positions.size());
			m.m_normals.resize(instance.mMesh->m_normals.size());
			m.m_indices = instance.mMesh->m_indices;

			for (int i = 0; i < int(instance.mMesh->m_positions.size()); ++i) {
				Vec3 softPos;
				Vec3 softNormal;

				for (int w = 0; w < 4; ++w) {
					const int cluster = instance.mSkinningIndices[i * 4 + w];
					const float weight = instance.mSkinningWeights[i * 4 + w];

					if (cluster > -1) {
						// offset in the global constraint array
						int rigidIndex = cluster + instance.mOffset;

						Vec3 localPos = Vec3(instance.mMesh->m_positions[i])
								- instance.mRigidRestPoses[cluster];

						Vec3 skinnedPos =
								g_buffers->rigidTranslations[rigidIndex]
										+ Rotate(
												g_buffers->rigidRotations[rigidIndex],
												localPos);
						Vec3 skinnedNormal = Rotate(
								g_buffers->rigidRotations[rigidIndex],
								instance.mMesh->m_normals[i]);

						softPos += skinnedPos * weight;
						softNormal += skinnedNormal * weight;
					}
				}

				m.m_positions[i] = Point3(softPos);
				m.m_normals[i] = softNormal;
			}

			DrawMesh(&m, instance.mColor);
		}
	}

};
