#include "Strand.h"
#include "json.hpp"
#include <fstream>

#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <direct.h>
#include <fileapi.h>
#endif

inline bool saveJson(std::string  filePath, nlohmann::json& j, int indent = -1) {
	std::ofstream ofs(filePath);
	if (ofs.is_open())
	{
		try
		{
			ofs << j.dump(indent) << std::endl;
			return true;
		}
		catch (nlohmann::json::exception& e)
		{
			std::cout << e.what() << '\n';
			return false;
		}
	}
	else
	{
		std::cout << "Fail to open: " << filePath << '\n';
		return false;
	}
}

bool folderExists(const char* folderName)
{
#ifdef _WIN32

	if (_access(folderName, 0) == -1) {
		//File not found
		return false;
	}

	DWORD attr = GetFileAttributes((LPCSTR)folderName);
	if (!(attr & FILE_ATTRIBUTE_DIRECTORY)) {
		// File is not a directory
		return false;
	}
#endif

	return true;
}


bool createFolder(std::string folderName)
{
#ifdef _WIN32
	std::list<std::string> folderLevels;
	char* c_str = (char*)folderName.c_str();

	// Point to end of the string
	char* strPtr = &c_str[strlen(c_str) - 1];

	// Create a list of the folders which do not currently exist
	do {
		if (folderExists(c_str)) {
			break;
		}
		// Break off the last folder name, store in folderLevels list
		do {
			strPtr--;
		} while ((*strPtr != '\\') && (*strPtr != '/') && (strPtr >= c_str));
		folderLevels.push_front(std::string(strPtr + 1));
		strPtr[1] = 0;
	} while (strPtr >= c_str);

	if (_chdir(c_str)) {
		return true;
	}

	// Create the folders iteratively
	for (std::list<std::string>::iterator it = folderLevels.begin(); it != folderLevels.end(); it++) {
		if (CreateDirectory(it->c_str(), NULL) == 0) {
			return true;
		}
		_chdir(it->c_str());
	}

#endif
	return false;
}

using namespace VBD;
struct SimulatorParams {
	SimulatorParams() {
		gravity << 0, -10, 0;
		//outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test4_20Verts_30degree_withSkip_stiffness1e6";

		//outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test5_3Verts_stiffnessRatio48";
		//outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test6_10Verts_stiffnessRatio48";
		//outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test7_10Verts_stiffnessRatio48_10substeps";
	}
	int numFrames = 300;
	int substeps = 1;
	int numIterations = 100;
	float dt = 0.01666666;
	float accelerationRho = 0.05;
	bool useAcceleration = false;
	Vec3 gravity;
	std::string outPath;
};

struct StrandSim
{
	void initializeHorizontal() {
		int numVerts = 3;
		float dis = 0.1f;
		float initHeight = 2.f;
		float stiffness = 1e6f;
		std::vector<Vec3> pos;
		std::vector<float> ls;
		float m0 = 1;
		float m1 = 1000;
		params.outPath = "C:\\Data\\Test4_20Verts_30degree_withSkip_stiffness1e8";

		std::vector<float> ms;

		for (size_t iV = 0; iV < numVerts; iV++)
		{
			Vec3 v;
			v << iV * dis, initHeight, 0.f;
			pos.push_back(v);
			if (iV < numVerts - 1)
			{
				ms.push_back(m0);
			}
			else
			{
				ms.push_back(m1);
			}
			if (iV > 0)
			{
				ls.push_back(dis);
			}
		}

		strand.from(pos, ms, ls, stiffness);

	}

	void initializeTilted() {
		int numVerts = 20;
		float dis = 0.05f;
		float initHeight = 2.f;
		float stiffness = 1e8f;
		std::vector<Vec3> pos;
		std::vector<float> ls;
		float m0 = 1;
		float m1 = 1000;
		bool addSkipSpring = true;
		float skipSpringStrength = 100;
		float tanAngle = 0.57735f; // 30 deg
		params.outPath = "C:\\Data\\Test4_20Verts_30degree_withSkip_stiffness1e8";

		std::vector<float> ms;

		for (size_t iV = 0; iV < numVerts; iV++)
		{
			Vec3 v;
			if (iV < numVerts - 1)
			{
				ms.push_back(m0);
				v << iV * dis, initHeight + iV * dis * tanAngle, 0.f;
			}
			else
			{
				ms.push_back(m1);
				v << (iV + 1) * dis, initHeight + (iV + 1) * dis * tanAngle, 0.f;

			}
			pos.push_back(v);
			if (iV > 0)
			{
				ls.push_back((pos.back() - pos[iV-1]).norm());
			}
		}
		strand.from(pos, ms, ls, stiffness, addSkipSpring, skipSpringStrength);

	}

	FloatingType getAcceleratorOmega(int order, CFloatingType pho, CFloatingType prevOmega)
	{
		switch (order)
		{
		case 1:
			return 1;
			break;
		case 2:
			return  2 / (2 - SQR(pho));
			break;
		default:
			assert(order > 0);

			return 4.f / (4 - SQR(pho) * prevOmega);;
			break;
		}
	}

	void applyAccelerator(FloatingType omega, TVerticesMat posBeforeIter) {
		if (omega > 1.f)
		{
			strand.mVertPos = omega * (strand.mVertPos - strand.prevprevPos) + strand.prevprevPos;
		}

		strand.prevprevPos = posBeforeIter;
	}

	void initializeStiffRatio() {
		//int numVerts = 3;
		//params.numIterations = 100;
		//params.accelerationRho = 0.0;
		//params.outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test5_3Verts_stiffnessRatio48";

		//int numVerts = 3;
		//params.numIterations = 20000;
		//params.accelerationRho = 0.0;
		//params.outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test5_3Verts_stiffnessRatio48_converged";

		//int numVerts = 10;
		//params.numIterations = 20000;
		//params.outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test6_10Verts_stiffnessRatio48_accelerated_converged";

		//int numVerts = 10;
		//params.numIterations = 100;
		//params.accelerationRho = 0.08;
		//params.outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test6_10Verts_stiffnessRatio48_accelerated";


		int numVerts = 5;
		params.numIterations = 100;
		params.accelerationRho = 0.0;
		params.outPath = "C:\\Data\\Test8_5Verts_stiffnessRatio48_accelerated";

		//int numVerts = 5;
		//params.numIterations = 20000;
		//params.accelerationRho = 0.0;
		//params.outPath = "E:\\Data2\\VBDSimulation\\S30_StrandTest\\Test8_5Verts_stiffnessRatio48_accelerated_converged";

		float dis = 0.05f;
		float initHeight = 2.f;
		float stiffness1 = 1e4f;
		float stiffness2 = 1e8f;
		std::vector<Vec3> pos;
		std::vector<float> ls;
		std::vector<float> stiffnesses;
		float m0 = 0.1;
		float m1 = 0.1;
		bool addSkipSpring = true;
		float skipSpringStrength = 100;
		float tanAngle = 0.57735f; // 30 deg

		std::vector<float> ms;

		for (size_t iV = 0; iV < numVerts; iV++)
		{
			Vec3 v;
			if (iV < numVerts - 1)
			{
				ms.push_back(m0);

				if (iV % 2)
				{
					stiffnesses.push_back(stiffness2);
				}
				else
				{
					stiffnesses.push_back(stiffness1);
				}
			}
			else
			{
				ms.push_back(m1);
			}
			v << iV * dis, initHeight + iV * dis * tanAngle, 0.f;

			pos.push_back(v);
			if (iV > 0)
			{
				ls.push_back((pos.back() - pos[iV - 1]).norm());
			}
			
		}
		
		strand.from(pos, ms, ls, stiffnesses);

	}

	void initialize() {
		initializeTilted();
		//initializeStiffRatio();

		outPath = params.outPath;
		createFolder(outPath);
	}

	void forwardStep() {
		strand.mVelocity.colwise() += params.dt * params.gravity;
		strand.mVelocity.col(0).setZero();;
		//std::cout << "mVelocity:\n" << strand.mVelocity.transpose() << "\n";

		strand.inertia = strand.mVertPos + strand.mVelocity * params.dt;

		strand.mVertPrevPos = strand.mVertPos;
		// initial step
		// strand.mVertPos = strand.inertia;

		TVerticesMat accelerationApprox;
		if (strand.hasVelocitiesPrev)
		{
			accelerationApprox = (strand.mVelocity - strand.mVelocitiesPrev) / params.dt;
			strand.hasApproxAcceleration = true;
		}

		FloatingType gravNorm = 10.f;
		Vec3 gravDir;
		gravDir << 0, -1, 0;

		
		FloatingType accelerationComponent = 0.f;
		if (strand.hasApproxAcceleration) {
			for (int iV = 0; iV < strand.numVerts; iV++)
			{
				accelerationComponent = accelerationApprox.col(iV).dot(gravDir);
				accelerationComponent = accelerationComponent < gravNorm ? accelerationComponent : gravNorm;
				accelerationComponent = accelerationComponent > 1e-5f ? accelerationComponent : 0.f;
				strand.mVertPos.col(iV) = strand.mVertPrevPos.col(iV) + params.dt * strand.mVelocitiesPrev.col(iV)
					+ params.dt * params.dt * gravDir * accelerationComponent;
			}
		}
		else
		{
				strand.mVertPos = strand.inertia;
		}
		//std::cout << "initial step:\n" << strand.mVertPos.transpose() << "\n";

	}

	void updateVelocity() {
		strand.mVelocitiesPrev = strand.mVelocity;
		strand.mVelocity = (strand.mVertPos - strand.mVertPrevPos) / params.dt;
		strand.mVelocity.col(0).setZero();

		strand.hasVelocitiesPrev = true;

	}

	void solve() {
		float dtSqrReciprocal = 1.f / (params.dt * params.dt);
		auto& edges = strand.edges;
		for (size_t iV = 1; iV < strand.numVerts; iV++)
		{
			Mat3 h = Mat3::Zero();
			Vec3 f = Vec3::Zero();

			f = strand.vertexMass(iV) * (strand.inertia.col(iV) - strand.vertex(iV))* (dtSqrReciprocal);
			h += strand.vertexMass(iV) * dtSqrReciprocal * Mat3::Identity();
			for (size_t iNeiE = 0; iNeiE < strand.vertAdjacentEdges[iV].size(); iNeiE++)
			{
				int edgeId1 = strand.vertAdjacentEdges[iV][iNeiE];

				if (edgeId1 != -1)
				{
					IdType v1 = edges[edgeId1][0];
					IdType v2 = edges[edgeId1][1];
					Vec3 diff = strand.mVertPos.col(v1) - strand.mVertPos.col(v2);
					FloatingType l = diff.norm();
					FloatingType l0 = strand.orgLengths(edgeId1);
					// evaluate hessian
					Mat3 h_1_1;
					FloatingType stiffness_ = strand.edgesStiffness[edgeId1];
					h_1_1 = stiffness_ * (Mat3::Identity() - (l0 / l) * (Mat3::Identity() - diff * diff.transpose() / (l * l)));
					h += h_1_1;

					if (v1 == iV)
					{
						f += (stiffness_ * (l0 - l) / l) * diff;

					}
					else {
						f -= (stiffness_ * (l0 - l) / l)* diff;
					}

				}
			}

			Vec3 dx = h.colPivHouseholderQr().solve(f);
			//std::cout << "dx for vertex iV" << iV << ":\n" << dx.transpose() << "\n";

			strand.vertex(iV) += dx;

		}
	}

	void simulate() {
		saveOutputs();
		params.dt = params.dt / params.substeps;

		for (frameId = 1; frameId < params.numFrames; frameId++)
		{
			for (step = 0; step < params.substeps; step++)
			{
				forwardStep();
				float omega=1.f;
				for (iter = 0; iter < params.numIterations; iter++)
				{
					TVerticesMat posBeforeIter = strand.mVertPos;
					solve();

					omega = getAcceleratorOmega(iter+1, params.accelerationRho, omega);
					if (iter % 10 == 0)
					{
						//std::cout << "acceleration ratio: " << omega << "\n";
					}

					if (params.useAcceleration)
					{
						applyAccelerator(omega, posBeforeIter);
					}
				}

				updateVelocity();

			}

			//std::cout << "Vertex position at frame:" << frameId << "\n" << strand.mVertPos.transpose() << "\n";
			std::cout << "Frame: " << frameId << " finished!\n";
			saveOutputs();
		}
	}

	void saveOutputs() {
		std::vector<std::array<FloatingType, 3>> verts;


		for (int iV = 0; iV < strand.numVerts; ++iV)
		{
			auto v = strand.mVertPos.col(iV);
			std::array<FloatingType, 3> pt = { v[0], v[1], v[2] };

			verts.push_back(pt);
		}

		std::ostringstream aSs;
		aSs << std::setfill('0') << std::setw(8) << frameId;
		std::string outNumber = aSs.str();
		std::string outFile = outPath + "/A" + outNumber + ".json";
		nlohmann::json j;
		j["pos"] = verts;
		saveJson(outFile, j, 2);
	}

	VBDStrand strand;
	SimulatorParams params;
	std::string outPath;
	int frameId = 0;
	int step = 0;
	int iter = 0;
};



int main(int argc, char** argv) {
	// simulateClothMeshStVK();
	StrandSim simulator;
	simulator.initialize();

	simulator.simulate();
}