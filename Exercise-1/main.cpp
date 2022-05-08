#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool checkVaildFace(Vertex* vertices, unsigned int idx0,unsigned int idx1, unsigned int idx2, float edgeThreshold){

	Vector4f vertexPosition0=vertices[idx0].position;
	Vector4f vertexPosition1=vertices[idx1].position;
	Vector4f vertexPosition2=vertices[idx2].position;
	
	if(vertexPosition0[0]==MINF || vertexPosition1[0]==MINF || vertexPosition2[0]==MINF) return false;

	float length01=(vertexPosition0-vertexPosition1).norm();
	float length02=(vertexPosition0-vertexPosition2).norm();
	float length12=(vertexPosition1-vertexPosition2).norm();
	if(length01>=edgeThreshold || length02>=edgeThreshold || length12>=edgeThreshold) return false;
	
	return true;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	
	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	std::vector<std::vector<unsigned int>> allValidFaceIdx;
	for(int idxY=0;idxY<height-1;idxY++){
		for(int idxX=0;idxX<width-1;idxX++){
			unsigned int currIdx0=idxY*width+idxX;
			unsigned int currIdx1=idxY*width+idxX+1;
			unsigned int currIdx2=(idxY+1)*width+idxX;
			unsigned int currIdx3=(idxY+1)*width+idxX+1;

			if(checkVaildFace(vertices,currIdx0,currIdx1,currIdx2,edgeThreshold)){
				allValidFaceIdx.push_back({currIdx0,currIdx1,currIdx2});
				nFaces++;
			}
			if(checkVaildFace(vertices,currIdx1,currIdx2,currIdx3,edgeThreshold)){
				allValidFaceIdx.push_back({currIdx1,currIdx2,currIdx3});
				nFaces++;
			}

		}
	}


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(int i=0;i<nVertices;i++){
		Vertex currVertex=vertices[i];
		Vector4f currVertexPosition=currVertex.position;
		Vector4uc currVertexColor=currVertex.color;
		if(currVertexPosition[0]==MINF){
			outFile<<"0.0 0.0 0.0 ";
		}else{
			outFile<<currVertexPosition[0]<<" "<<currVertexPosition[1]<<" "<<currVertexPosition[2]<<" ";
		}
		outFile<<(int)currVertexColor[0]<<" "<<(int)currVertexColor[1]<<" "<<(int)currVertexColor[2]<<" "<<(int)currVertexColor[3]<<std::endl;

	}


	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	for(auto currValidFaceIdx:allValidFaceIdx){
		outFile<<"3 "<< currValidFaceIdx[0] << " " << currValidFaceIdx[1] << " "<< currValidFaceIdx[2] << " " << std::endl;
	}


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		int Width=sensor.GetDepthImageWidth();
		int Height=sensor.GetDepthImageHeight();
		int AllPixels= Width*Height;
		for(int idx=0;idx<AllPixels;idx++){
			int currHeight=idx/Width;
			int currWidth=idx%Width;
			float currDepth=depthMap[idx];

			if(currDepth==MINF){
				vertices[idx].position=Vector4f(MINF, MINF, MINF, MINF);
				vertices[idx].color = Vector4uc(0,0,0,0);
			}else{
				Vector3f CurrPoint3DAfterIntrin=currDepth*depthIntrinsicsInv*Vector3f(currWidth,currHeight,1);
				vertices[idx].position=trajectoryInv*depthExtrinsicsInv*CurrPoint3DAfterIntrin.homogeneous();
				vertices[idx].color=Vector4uc(colorMap[4*idx], colorMap[4*idx+1], colorMap[4*idx+2], colorMap[4*idx+3]);
			}

		}
		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}