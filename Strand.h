#pragma once
#include "Types.h"


namespace VBD {

	struct Mesh
	{
		TVerticesMat mVertPos;
		TVerticesMat mVertPrevPos;
		TVerticesMat mVelocity;
		VecDynamic vertexMass;

		Vec3Block vertex(int i) {
			return mVertPos.block<3, 1>(0, i);
		}
	};

	struct VBDStrand : public Mesh
	{
	public:
		std::vector<Vec2I> edges;
		std::vector<FloatingType> edgesStiffness;
		std::vector<std::vector<int>> vertAdjacentEdges;
		size_t nEdges;
		FloatingType stiffness;
		FloatingType stiffnessSkipSpring;
		VecDynamic orgLengths;
		TVerticesMat mVelocitiesPrev;
		TVerticesMat inertia;
		size_t numVerts;
		
		TVerticesMat prevprevPos;
		bool hasVelocitiesPrev = false;
		bool hasApproxAcceleration = false;
		void from(const std::vector<Vec3> & pos, 
			const std::vector<FloatingType>& masses, 
			const std::vector<FloatingType>& lengthes,
			FloatingType stiffness_in, 
			bool addSkipSpring=false,
			FloatingType stiffnessSkipSpring_in = 1e2
		) {

			stiffness = stiffness_in;
			stiffnessSkipSpring = stiffnessSkipSpring_in;

			numVerts = pos.size();
			assert(masses.size() == lengthes.size() + 1);
			assert(pos.size() == lengthes.size() + 1);

	
			vertexMass = VecDynamic::Map(&masses[0], masses.size());


			vertAdjacentEdges.resize( pos.size());
			std::vector<FloatingType> lengthesNew;
			for (size_t i = 0; i < lengthes.size(); ++i) {
				Vec2I edge;
				edge << i, i + 1;
				int edgeId = edges.size();
				edges.push_back(edge);

				vertAdjacentEdges[i].push_back(edgeId);
				vertAdjacentEdges[i+1].push_back(edgeId);
				edgesStiffness.push_back(stiffness);
				lengthesNew.push_back(lengthes[i]);
				if (addSkipSpring && i < lengthes.size() - 1)
				{
					edgeId = edges.size();
					edge << i, i + 2;
					edges.push_back(edge);
					vertAdjacentEdges[i].push_back(edgeId);
					vertAdjacentEdges[i + 2].push_back(edgeId);
					edgesStiffness.push_back(stiffnessSkipSpring);
					lengthesNew.push_back(lengthes[i] + lengthes[i+1]);
				}
			}

			orgLengths = VecDynamic::Map(&lengthesNew[0], lengthesNew.size());


			mVertPos.resize(3, pos.size());
			mVertPos.setZero();
			for (size_t i = 0; i < pos.size(); ++i) {
				mVertPos.col(i) = pos[i];
			}
			mVertPrevPos = mVertPos;
			mVelocity.resizeLike(mVertPos);
			mVelocity.setZero();
			mVelocitiesPrev.resizeLike(mVertPos);
			mVelocitiesPrev.setZero();

		}

		void from(const std::vector<Vec3>& pos,
			const std::vector<FloatingType>& masses,
			const std::vector<FloatingType>& lengthes,
			std::vector<FloatingType> stiffness_in
		) {

			numVerts = pos.size();
			assert(masses.size() == lengthes.size() + 1);
			assert(pos.size() == lengthes.size() + 1);
			edgesStiffness = stiffness_in;

			vertexMass = VecDynamic::Map(&masses[0], masses.size());

			vertAdjacentEdges.resize(pos.size());
			std::vector<FloatingType> lengthesNew;
			for (size_t i = 0; i < lengthes.size(); ++i) {
				Vec2I edge;
				edge << i, i + 1;
				int edgeId = edges.size();
				edges.push_back(edge);

				vertAdjacentEdges[i].push_back(edgeId);
				vertAdjacentEdges[i + 1].push_back(edgeId);
				edgesStiffness.push_back(stiffness);
				lengthesNew.push_back(lengthes[i]);
			}

			orgLengths = VecDynamic::Map(&lengthesNew[0], lengthesNew.size());


			mVertPos.resize(3, pos.size());
			mVertPos.setZero();
			for (size_t i = 0; i < pos.size(); ++i) {
				mVertPos.col(i) = pos[i];
			}
			mVertPrevPos = mVertPos;
			mVelocity.resizeLike(mVertPos);
			mVelocity.setZero();
			mVelocitiesPrev.resizeLike(mVertPos);
			mVelocitiesPrev.setZero();

		}
	};

}