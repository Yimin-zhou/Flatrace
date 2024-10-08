/*
Supplemental C/C++ implementation of the DiTO OBB construction method described in the
chapter "Fast Computation of Tight-Fitting Oriented Bounding Boxes" of the book "Game 
Engine Gems 2".
*/

/*
This source file is distributed under the following BSD license:

Copyright 2011 Thomas Larsson and Linus Kallberg. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THOMAS LARSSON AND LINUS KALLBERG ``AS IS'' AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THOMAS LARSSON, LINUS 
KALLBERG, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Header file defining the interface to the DiTO-14 implementation.
Version: 1.0
*/

#ifndef DITO_H_INCLUSION_GUARD
#define DITO_H_INCLUSION_GUARD

#include <glm/glm.hpp>
#include <Eigen/Dense>

#include "src/core/types.h"

namespace DiTO
{

/*
Representation of a 3-dimensional vector. All vector structures with the same
memory layout can be used directly as input to the algorithm by
first casting the array pointer to DiTO::Vector<F>* (where, e.g., F = float or F = double).
*/
    template<typename F>
    struct Vector
    {
        Vector(F x, F y, F z) : x(x), y(y), z(z) {}

        Vector() : x(0), y(0), z(0) {}

        F x, y, z;
    };

/*
Representation of an oriented bounding box.
Members:
	mid - The midpoint, expressed in the standard base
	v0, v1, v2 - Orthogonal vectors defining the orientation
	ext - Absolute values of the extents along the three vectors, measured from mid
*/
    template<typename F>
    struct OBB
    {
        Vector<F> mid;
        Vector<F> v0, v1, v2;
        Vector<F> ext;
        glm::mat4 invMatrix;

        // Calculate the surface area of an OBB
        const double area() const
        {
            return 8.0f * (this->ext.x * this->ext.y + this->ext.y * this->ext.z + this->ext.z * this->ext.x);
        }
    };

    // Flatten an OBB into a vector
    template<typename F>
    Eigen::Matrix<F, Eigen::Dynamic, 1> flatten(const OBB<F> &obb)
    {
        Eigen::Matrix<F, Eigen::Dynamic, 1> flattened(12); // 4 vectors of 3 components each

        flattened << obb.v0.x, obb.v0.y, obb.v0.z,
                obb.v1.x, obb.v1.y, obb.v1.z,
                obb.v2.x, obb.v2.y, obb.v2.z,
                obb.ext.x, obb.ext.y, obb.ext.z;

        return flattened;
    }

/*
Compute an oriented bounding box using the DiTO-14 algorithm.
Parameters:
	vertArr - The array of vertices
	nv - The number of vertices
	obb - Output parameter for the resulting OBB
Postcondition: vertArr is unchanged, and obb holds the computed OBB
The function has explicit instantiations with F = float and F = double at the
top of the .cpp file.
*/
    template<typename F>
    extern void DiTO_14(Vector<F> vertArr[], int nv, OBB<F> &obb);

}

#endif    // ifndef DITO_H_INCLUSION_GUARD
