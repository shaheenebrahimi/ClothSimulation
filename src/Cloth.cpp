#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double stiffness)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;
	
	//
	// Create particles here
	//
	this->n = 0; // size of global vector (do not count fixed vertices)
	double radius = 0.01; // Used for collisions
	int nVerts = rows*cols;
    Vector3d xStep = (x01 - x00) / double(cols-1);
    Vector3d yStep = (x10 - x00) / double(rows-1);

	for(int r = 0; r < rows; ++r) {
		for(int c = 0; c < cols; ++c) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			
            // Compute particle member variables
            p->r = radius;
            p->i = ((r == 0 && c == 0) || (r == 0 && c == cols - 1)) ? -1 : n++;
            p->x0 = x00 + c * xStep + r * yStep;
            p->v0 = Vector3d(0,0,0);
            p->x = p->x0;
            p->v = p->v0;
            p->fixed = p->i == -1;
            p->m = mass / double(nVerts);
		}
	}
    
	
	//
	// Create springs here
	//
    for(int r = 0; r < rows; ++r) {
        for(int c = 0; c < cols; ++c) {
            int i = r*cols + c;
            auto particle = particles[i];
            
            // X & Y Springs
            if (c+1 < cols) // +x
                springs.push_back(createSpring(particle, particles[r*cols + (c+1)], stiffness));
                
            if (r+1 < rows) // +y
                springs.push_back(createSpring(particle, particles[(r+1)*cols + c], stiffness));
            
            // Shear Springs
            if (c-1 >= 0 && r+1 < rows) // -x +y
                springs.push_back(createSpring(particle, particles[(r+1)*cols + (c-1)], stiffness));
            if (c+1 < cols && r+1 < rows) // +x +y
                springs.push_back(createSpring(particle, particles[(r+1)*cols + (c+1)], stiffness));

            // Bending Springs
            if (c+2 < cols) // +2x
                springs.push_back(createSpring(particle, particles[r*cols + (c+2)], stiffness));
            if (r+2 < rows) // +2y
                springs.push_back(createSpring(particle, particles[(r+2)*cols + c], stiffness));
        }
    }
    
    

	// Allocate system matrices and vectors
	M.resize(3*n,3*n);
	K.resize(3*n,3*n);
	v.resize(3*n);
	f.resize(3*n);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();

	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}

	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}
}

void Cloth::step(double h, const Vector3d &grav, const vector<shared_ptr<Particle>> spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	//
	// Step
    //
    
    // Sparse Matrix Triplets
    std::vector<Eigen::Triplet<double>> MList;
    std::vector<Eigen::Triplet<double>> KList;
    
    // Populate initial values
    for (auto particle : particles) {
        if (particle->fixed) continue;
        int i = particle->i;
        
//        M.block<3,3>(3*i,3*i) = particle->m * Matrix3d::Identity();
        for (int axis = 0; axis < 3; ++axis)
            MList.push_back(Eigen::Triplet<double>(3*i + axis, 3*i + axis, particle->m));
        f.segment<3>(3*i) = particle->m * grav;
        v.segment<3>(3*i) = particle->v;
    }
    
    // Add spring forces
    for (auto spring : springs) {
        // Intermediary values
        Vector3d dx = spring->p1->x - spring->p0->x;
        double l = dx.norm();
        double dl = (l - spring->L) / l;
        Matrix3d lhs = dx * dx.transpose();
        Matrix3d rhs = (dx.transpose() * dx) * Matrix3d::Identity();
        
        // Compute fs and Ks
        Vector3d fs = spring->E * (l-spring->L) * (dx/l);
        Matrix3d Ks = (spring->E/(l*l)) * ((1-dl)*lhs + dl*rhs);
        
        // Apply forces if not fixed
        if (!spring->p0->fixed) {
            f.segment<3>(3*spring->p0->i) += fs;
            for (int xAxis = 0; xAxis < 3; ++xAxis) {
                for (int yAxis = 0; yAxis < 3; ++yAxis) {
                    KList.push_back(Eigen::Triplet<double>(3*spring->p0->i + xAxis, 3*spring->p0->i + yAxis, -Ks(xAxis, yAxis)));
                }
            }
        }
        if (!spring->p1->fixed) {
            f.segment<3>(3*spring->p1->i) -= fs;
            for (int xAxis = 0; xAxis < 3; ++xAxis) {
                for (int yAxis = 0; yAxis < 3; ++yAxis) {
                    if (Ks(xAxis, yAxis) == 0) continue;
                    KList.push_back(Eigen::Triplet<double>(3*spring->p1->i + xAxis, 3*spring->p1->i + yAxis, -Ks(xAxis, yAxis)));
                }
            }
        }
        if (!spring->p0->fixed && !spring->p1->fixed) {
            for (int xAxis = 0; xAxis < 3; ++xAxis) {
                for (int yAxis = 0; yAxis < 3; ++yAxis) {
                    if (Ks(xAxis, yAxis) == 0) continue;
                    KList.push_back(Eigen::Triplet<double>(3*spring->p0->i + xAxis, 3*spring->p1->i + yAxis, Ks(xAxis, yAxis)));
                    KList.push_back(Eigen::Triplet<double>(3*spring->p1->i + xAxis, 3*spring->p0->i + yAxis, Ks(xAxis, yAxis)));
                }
            }
        }
        
    }
    
    // Resolve Collisions
    for (auto sphere : spheres) {
        // Check collisions for each sphere
        for (auto particle : particles) {
            if (particle->fixed) continue;
            Vector3d dx = particle->x - sphere->x;
            double l = dx.norm();
            Vector3d n = dx / l; // collision normal
            double d = particle->r + sphere->r - l; // penetration depth
            if (d > 0) { // collision detected
                double c = 10;
                Vector3d fc = c * d * n;
                Matrix3d Kc = c * d * Matrix3d::Identity();
                f.segment<3>(3*particle->i) += fc;
//                K.block<3,3>(3*particle->i, 3*particle->i) -= Kc;
                for (int xAxis = 0; xAxis < 3; ++xAxis) {
                    for (int yAxis = 0; yAxis < 3; ++yAxis) {
                        if (Kc(xAxis, yAxis) == 0) continue;
                        KList.push_back(Eigen::Triplet<double>(3*particle->i + xAxis, 3*particle->i + yAxis, -Kc(xAxis, yAxis)));
                    }
                }
            }
        }
    }
    
    // Set Sparse Matrix from triplets
    M.setFromTriplets(MList.begin(), MList.end());
    K.setFromTriplets(KList.begin(), KList.end(), [](double a, double b) { return a + b; });
    
    // Solve system of equations for v^t+1
    SparseMatrix<double> A = M - h*h*K;
    VectorXd b = M*v + h*f;
    ConjugateGradient< SparseMatrix<double> > cg;
    cg.setMaxIterations(25);
    cg.setTolerance(1e-6);
    cg.compute(A);
    VectorXd x = cg.solveWithGuess(b, v); // set initial guess to current velocity vector

    // Extract values
    for (auto particle : particles) {
        if (particle->fixed) continue;

        particle->v = x.segment<3>(3*particle->i);
        particle->x += h * particle->v;
    }
	
	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
