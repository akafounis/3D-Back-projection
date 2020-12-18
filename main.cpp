#include <iostream>
#include <fstream>
#include <array>
#include <math.h>
#include "Eigen.h"
#include <list>
#include <iterator>
#include "VirtualSensor.h"


struct Vertex {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // position stored as 4 floats (4th component is supposed to be 1.0)
    Vector4f position;

    // color stored as 4 unsigned char
    Vector4uc color;
};


bool WriteMesh(Vertex *vertices, unsigned int width, unsigned int height, const std::string &filename) {
    float edgeThreshold = 0.01f; // 1cm

    // Get number of vertices
    unsigned int nVertices = width * height;


    // Determine number of valid faces
    unsigned nFaces = 0;

    std::list <std::string> faces;

    // count the number of faces
    // and store them in an array to write them later in the .off file
    for (int i = 0; i < width * height; i++) {
        int i1 = i;
        int i2 = i + 1;
        int i3 = width + i2;
        if (i3 > width * height) {
            continue;
        }
        if (vertices[i1].position.x() == vertices[i1].position.x() &&
            vertices[i2].position.x() == vertices[i2].position.x() &&
            vertices[i3].position.x() == vertices[i3].position.x()) {
            float x1 = vertices[i1].position.x();
            float x2 = vertices[i2].position.x();
            float x3 = vertices[i3].position.x();
            float y1 = vertices[i1].position.y();
            float y2 = vertices[i2].position.y();
            float y3 = vertices[i3].position.y();
            float z1 = vertices[i1].position.z();
            float z2 = vertices[i2].position.z();
            float z3 = vertices[i3].position.z();
            float edge1 = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
            float edge2 = sqrt(pow(x1 - x3, 2) + pow(y1 - y3, 2) + pow(z1 - z3, 2));
            float edge3 = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2) + pow(z3 - z2, 2));
            if (edge1 <= edgeThreshold && edge2 <= edgeThreshold && edge3 <= edgeThreshold) {
                nFaces++;
                std::stringstream f;
                f << 3 << " " << i1 << " " << i2 << " " << i3 << std::endl;
                faces.push_back(f.str());
            }
        }

        // for the lower triangles
        // just have to change the i2 index
        i2 = width + i1;
        if (i2 > width * height) {
            continue;
        }
        if (vertices[i1].position.x() == vertices[i1].position.x() &&
            vertices[i2].position.x() == vertices[i2].position.x() &&
            vertices[i3].position.x() == vertices[i3].position.x()) {
            float x1 = vertices[i1].position.x();
            float x2 = vertices[i2].position.x();
            float x3 = vertices[i3].position.x();
            float y1 = vertices[i1].position.y();
            float y2 = vertices[i2].position.y();
            float y3 = vertices[i3].position.y();
            float z1 = vertices[i1].position.z();
            float z2 = vertices[i2].position.z();
            float z3 = vertices[i3].position.z();
            float edge1 = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
            float edge2 = sqrt(pow(x1 - x3, 2) + pow(y1 - y3, 2) + pow(z1 - z3, 2));
            float edge3 = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2) + pow(z3 - z2, 2));
            if (edge1 <= edgeThreshold && edge2 <= edgeThreshold && edge3 <= edgeThreshold) {
                nFaces++;
                std::stringstream f;
                f << 3 << " " << i1 << " " << i2 << " " << i3 << std::endl;
                faces.push_back(f.str());
            }
        }
    }



    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;




    // save vertices
    for (int i = 0; i < (width * height); i++) {
        if (vertices[i].position.x() == vertices[i].position.x()) {
            outFile << vertices[i].position.x() << " " << vertices[i].position.y() << " " << vertices[i].position.z()
                    << " ";
            outFile << (int) vertices[i].color.x() << " " << (int) vertices[i].color.y() << " "
                    << (int) vertices[i].color.z() << " " << (int) vertices[i].color.w() << std::endl;
        } else {
            outFile << "0 0 0 0 0 0 0" << std::endl;
        }
    }


    // save valid faces
    for (auto & face : faces){
        outFile <<  face << std::endl;
    }

    // close file
    outFile.close();

    return true;

}

int main() {
    // Make sure this path points to the data folder
    std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn)) {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    // convert video to meshes
    while (sensor.ProcessNextFrame()) {

        // get ptr to the current depth frame
        // depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
        float *depthMap = sensor.GetDepth();

        float *pixValue;

        // get ptr to the current color frame
        // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
        BYTE *colorMap = sensor.GetColorRGBX();

        // get depth intrinsics
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        float fX = depthIntrinsics(0, 0);
        float fY = depthIntrinsics(1, 1);
        float cX = depthIntrinsics(0, 2);
        float cY = depthIntrinsics(1, 2);

        Matrix3f depthIntrinsicsInverse = sensor.GetDepthIntrinsics().inverse();

        // compute inverse depth extrinsics
        Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

        Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();


        // Back-projection
        Vertex *vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

        float X;
        float Y;
        float Z;
        int idx = 0;
        for (int i = 0; i < sensor.GetDepthImageHeight(); i++) {
            for (int j = 0; j < sensor.GetDepthImageWidth(); j++) {
                Vertex v;
                Z = depthMap[idx];
                if (isinf(Z)) {
                    v.position = Vector4f(MINF, MINF, MINF, MINF);
                    v.color = Vector4uc(0, 0, 0, 0);
                } else {
                    X = (j - cX) * Z / fX;
                    Y = (i - cY) * Z / fY;
                    Vector4f pos = Vector4f(X, Y, Z, 1.0);
                    v.position = pos;
                    Vector4uc color;
                    color = Vector4uc(((int) colorMap[idx * 4]), ((int) colorMap[idx * 4 + 1]),
                                      ((int) colorMap[idx * 4 + 2]), ((int) colorMap[idx * 4 + 3]));
                    v.color = color;
                }
                v.position = trajectoryInv * v.position;
                vertices[idx] = v;
                idx++;
            }
        }


        // write mesh file
        std::stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        std::cout << "JUST WROTE TO THE FILE: " << ss.str() << std::endl;
        if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str())) {
            std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            return -1;
        }

        // free mem
        delete[] vertices;
    }

    return 0;
}

