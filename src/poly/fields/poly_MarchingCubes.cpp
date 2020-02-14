//-----------------------------------------------------------------------------
// Created on: 27 December 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

// Poly includes
#include <mobius/poly_MarchingCubes.h>

//-----------------------------------------------------------------------------

//! For any edge, if one vertex is inside of the surface and the other is
//! outside of the surface, then the edge intersects the surface. For each of
//! the 8 vertices of the cube, there can be two possible states: either inside
//! or outside of the surface. Therefore, for any cube, the are 2^8=256 possible
//! sets of vertex states.
//!
//! This table lists the edges intersected by the surface for all 256 possible
//! vertex states. There are 12 edges. For each entry in the table, if edge #n
//! is intersected, then bit #n is set to 1. E.g.,
//!
//! 0x406 = 0100 0000 0110
//!
//! i.e., among 12 edges (12 bits), three edges are activated, i.e., there is
//! intersection at these edges.
const int aiCubeEdgeFlags[256] =
{
  0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
  0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
  0x230, 0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
  0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
  0x460, 0x569, 0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
  0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
  0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
  0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
  0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
  0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
  0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
  0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x066, 0x76a, 0x663, 0x569, 0x460,
  0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0,
  0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x033, 0x339, 0x230,
  0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190,
  0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x000
};

//!  For each of the possible vertex states listed in aiCubeEdgeFlags there is a specific triangulation
//!  of the edge intersection points.  a2iTriangleConnectionTable lists all of them in the form of
//!  0-5 edge triples with the list terminated by the invalid value -1.
//!
//! For example: a2iTriangleConnectionTable[3] list the 2 triangles formed when corner[0] 
//! and corner[1] are inside of the surface, but the rest of the cube is not.
const int a2iTriangleConnectionTable[256][16] =
{
  {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  1,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  8,  3,  9,  8,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3,  1,  2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  2, 10,  0,  2,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 2,  8,  3,  2, 10,  8, 10,  9,  8, -1, -1, -1, -1, -1, -1, -1},
  { 3, 11,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0, 11,  2,  8, 11,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  9,  0,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1, 11,  2,  1,  9, 11,  9,  8, 11, -1, -1, -1, -1, -1, -1, -1},
  { 3, 10,  1, 11, 10,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0, 10,  1,  0,  8, 10,  8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
  { 3,  9,  0,  3, 11,  9, 11, 10,  9, -1, -1, -1, -1, -1, -1, -1},
  { 9,  8, 10, 10,  8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  3,  0,  7,  3,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  1,  9,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  1,  9,  4,  7,  1,  7,  3,  1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  4,  7,  3,  0,  4,  1,  2, 10, -1, -1, -1, -1, -1, -1, -1},
  { 9,  2, 10,  9,  0,  2,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1},
  { 2, 10,  9,  2,  9,  7,  2,  7,  3,  7,  9,  4, -1, -1, -1, -1},
  { 8,  4,  7,  3, 11,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {11,  4,  7, 11,  2,  4,  2,  0,  4, -1, -1, -1, -1, -1, -1, -1},
  { 9,  0,  1,  8,  4,  7,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1},
  { 4,  7, 11,  9,  4, 11,  9, 11,  2,  9,  2,  1, -1, -1, -1, -1},
  { 3, 10,  1,  3, 11, 10,  7,  8,  4, -1, -1, -1, -1, -1, -1, -1},
  { 1, 11, 10,  1,  4, 11,  1,  0,  4,  7, 11,  4, -1, -1, -1, -1},
  { 4,  7,  8,  9,  0, 11,  9, 11, 10, 11,  0,  3, -1, -1, -1, -1},
  { 4,  7, 11,  4, 11,  9,  9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
  { 9,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  5,  4,  0,  8,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  5,  4,  1,  5,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 8,  5,  4,  8,  3,  5,  3,  1,  5, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10,  9,  5,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  0,  8,  1,  2, 10,  4,  9,  5, -1, -1, -1, -1, -1, -1, -1},
  { 5,  2, 10,  5,  4,  2,  4,  0,  2, -1, -1, -1, -1, -1, -1, -1},
  { 2, 10,  5,  3,  2,  5,  3,  5,  4,  3,  4,  8, -1, -1, -1, -1},
  { 9,  5,  4,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0, 11,  2,  0,  8, 11,  4,  9,  5, -1, -1, -1, -1, -1, -1, -1},
  { 0,  5,  4,  0,  1,  5,  2,  3, 11, -1, -1, -1, -1, -1, -1, -1},
  { 2,  1,  5,  2,  5,  8,  2,  8, 11,  4,  8,  5, -1, -1, -1, -1},
  {10,  3, 11, 10,  1,  3,  9,  5,  4, -1, -1, -1, -1, -1, -1, -1},
  { 4,  9,  5,  0,  8,  1,  8, 10,  1,  8, 11, 10, -1, -1, -1, -1},
  { 5,  4,  0,  5,  0, 11,  5, 11, 10, 11,  0,  3, -1, -1, -1, -1},
  { 5,  4,  8,  5,  8, 10, 10,  8, 11, -1, -1, -1, -1, -1, -1, -1},
  { 9,  7,  8,  5,  7,  9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  3,  0,  9,  5,  3,  5,  7,  3, -1, -1, -1, -1, -1, -1, -1},
  { 0,  7,  8,  0,  1,  7,  1,  5,  7, -1, -1, -1, -1, -1, -1, -1},
  { 1,  5,  3,  3,  5,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  7,  8,  9,  5,  7, 10,  1,  2, -1, -1, -1, -1, -1, -1, -1},
  {10,  1,  2,  9,  5,  0,  5,  3,  0,  5,  7,  3, -1, -1, -1, -1},
  { 8,  0,  2,  8,  2,  5,  8,  5,  7, 10,  5,  2, -1, -1, -1, -1},
  { 2, 10,  5,  2,  5,  3,  3,  5,  7, -1, -1, -1, -1, -1, -1, -1},
  { 7,  9,  5,  7,  8,  9,  3, 11,  2, -1, -1, -1, -1, -1, -1, -1},
  { 9,  5,  7,  9,  7,  2,  9,  2,  0,  2,  7, 11, -1, -1, -1, -1},
  { 2,  3, 11,  0,  1,  8,  1,  7,  8,  1,  5,  7, -1, -1, -1, -1},
  {11,  2,  1, 11,  1,  7,  7,  1,  5, -1, -1, -1, -1, -1, -1, -1},
  { 9,  5,  8,  8,  5,  7, 10,  1,  3, 10,  3, 11, -1, -1, -1, -1},
  { 5,  7,  0,  5,  0,  9,  7, 11,  0,  1,  0, 10, 11, 10,  0, -1},
  {11, 10,  0, 11,  0,  3, 10,  5,  0,  8,  0,  7,  5,  7,  0, -1},
  {11, 10,  5,  7, 11,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {10,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  0,  1,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  8,  3,  1,  9,  8,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1},
  { 1,  6,  5,  2,  6,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  6,  5,  1,  2,  6,  3,  0,  8, -1, -1, -1, -1, -1, -1, -1},
  { 9,  6,  5,  9,  0,  6,  0,  2,  6, -1, -1, -1, -1, -1, -1, -1},
  { 5,  9,  8,  5,  8,  2,  5,  2,  6,  3,  2,  8, -1, -1, -1, -1},
  { 2,  3, 11, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {11,  0,  8, 11,  2,  0, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1},
  { 0,  1,  9,  2,  3, 11,  5, 10,  6, -1, -1, -1, -1, -1, -1, -1},
  { 5, 10,  6,  1,  9,  2,  9, 11,  2,  9,  8, 11, -1, -1, -1, -1},
  { 6,  3, 11,  6,  5,  3,  5,  1,  3, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8, 11,  0, 11,  5,  0,  5,  1,  5, 11,  6, -1, -1, -1, -1},
  { 3, 11,  6,  0,  3,  6,  0,  6,  5,  0,  5,  9, -1, -1, -1, -1},
  { 6,  5,  9,  6,  9, 11, 11,  9,  8, -1, -1, -1, -1, -1, -1, -1},
  { 5, 10,  6,  4,  7,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  3,  0,  4,  7,  3,  6,  5, 10, -1, -1, -1, -1, -1, -1, -1},
  { 1,  9,  0,  5, 10,  6,  8,  4,  7, -1, -1, -1, -1, -1, -1, -1},
  {10,  6,  5,  1,  9,  7,  1,  7,  3,  7,  9,  4, -1, -1, -1, -1},
  { 6,  1,  2,  6,  5,  1,  4,  7,  8, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2,  5,  5,  2,  6,  3,  0,  4,  3,  4,  7, -1, -1, -1, -1},
  { 8,  4,  7,  9,  0,  5,  0,  6,  5,  0,  2,  6, -1, -1, -1, -1},
  { 7,  3,  9,  7,  9,  4,  3,  2,  9,  5,  9,  6,  2,  6,  9, -1},
  { 3, 11,  2,  7,  8,  4, 10,  6,  5, -1, -1, -1, -1, -1, -1, -1},
  { 5, 10,  6,  4,  7,  2,  4,  2,  0,  2,  7, 11, -1, -1, -1, -1},
  { 0,  1,  9,  4,  7,  8,  2,  3, 11,  5, 10,  6, -1, -1, -1, -1},
  { 9,  2,  1,  9, 11,  2,  9,  4, 11,  7, 11,  4,  5, 10,  6, -1},
  { 8,  4,  7,  3, 11,  5,  3,  5,  1,  5, 11,  6, -1, -1, -1, -1},
  { 5,  1, 11,  5, 11,  6,  1,  0, 11,  7, 11,  4,  0,  4, 11, -1},
  { 0,  5,  9,  0,  6,  5,  0,  3,  6, 11,  6,  3,  8,  4,  7, -1},
  { 6,  5,  9,  6,  9, 11,  4,  7,  9,  7, 11,  9, -1, -1, -1, -1},
  {10,  4,  9,  6,  4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4, 10,  6,  4,  9, 10,  0,  8,  3, -1, -1, -1, -1, -1, -1, -1},
  {10,  0,  1, 10,  6,  0,  6,  4,  0, -1, -1, -1, -1, -1, -1, -1},
  { 8,  3,  1,  8,  1,  6,  8,  6,  4,  6,  1, 10, -1, -1, -1, -1},
  { 1,  4,  9,  1,  2,  4,  2,  6,  4, -1, -1, -1, -1, -1, -1, -1},
  { 3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4, -1, -1, -1, -1},
  { 0,  2,  4,  4,  2,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 8,  3,  2,  8,  2,  4,  4,  2,  6, -1, -1, -1, -1, -1, -1, -1},
  {10,  4,  9, 10,  6,  4, 11,  2,  3, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  2,  2,  8, 11,  4,  9, 10,  4, 10,  6, -1, -1, -1, -1},
  { 3, 11,  2,  0,  1,  6,  0,  6,  4,  6,  1, 10, -1, -1, -1, -1},
  { 6,  4,  1,  6,  1, 10,  4,  8,  1,  2,  1, 11,  8, 11,  1, -1},
  { 9,  6,  4,  9,  3,  6,  9,  1,  3, 11,  6,  3, -1, -1, -1, -1},
  { 8, 11,  1,  8,  1,  0, 11,  6,  1,  9,  1,  4,  6,  4,  1, -1},
  { 3, 11,  6,  3,  6,  0,  0,  6,  4, -1, -1, -1, -1, -1, -1, -1},
  { 6,  4,  8, 11,  6,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 7, 10,  6,  7,  8, 10,  8,  9, 10, -1, -1, -1, -1, -1, -1, -1},
  { 0,  7,  3,  0, 10,  7,  0,  9, 10,  6,  7, 10, -1, -1, -1, -1},
  {10,  6,  7,  1, 10,  7,  1,  7,  8,  1,  8,  0, -1, -1, -1, -1},
  {10,  6,  7, 10,  7,  1,  1,  7,  3, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2,  6,  1,  6,  8,  1,  8,  9,  8,  6,  7, -1, -1, -1, -1},
  { 2,  6,  9,  2,  9,  1,  6,  7,  9,  0,  9,  3,  7,  3,  9, -1},
  { 7,  8,  0,  7,  0,  6,  6,  0,  2, -1, -1, -1, -1, -1, -1, -1},
  { 7,  3,  2,  6,  7,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 2,  3, 11, 10,  6,  8, 10,  8,  9,  8,  6,  7, -1, -1, -1, -1},
  { 2,  0,  7,  2,  7, 11,  0,  9,  7,  6,  7, 10,  9, 10,  7, -1},
  { 1,  8,  0,  1,  7,  8,  1, 10,  7,  6,  7, 10,  2,  3, 11, -1},
  {11,  2,  1, 11,  1,  7, 10,  6,  1,  6,  7,  1, -1, -1, -1, -1},
  { 8,  9,  6,  8,  6,  7,  9,  1,  6, 11,  6,  3,  1,  3,  6, -1},
  { 0,  9,  1, 11,  6,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 7,  8,  0,  7,  0,  6,  3, 11,  0, 11,  6,  0, -1, -1, -1, -1},
  { 7, 11,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 7,  6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  0,  8, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  1,  9, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 8,  1,  9,  8,  3,  1, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1},
  {10,  1,  2,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10,  3,  0,  8,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1},
  { 2,  9,  0,  2, 10,  9,  6, 11,  7, -1, -1, -1, -1, -1, -1, -1},
  { 6, 11,  7,  2, 10,  3, 10,  8,  3, 10,  9,  8, -1, -1, -1, -1},
  { 7,  2,  3,  6,  2,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 7,  0,  8,  7,  6,  0,  6,  2,  0, -1, -1, -1, -1, -1, -1, -1},
  { 2,  7,  6,  2,  3,  7,  0,  1,  9, -1, -1, -1, -1, -1, -1, -1},
  { 1,  6,  2,  1,  8,  6,  1,  9,  8,  8,  7,  6, -1, -1, -1, -1},
  {10,  7,  6, 10,  1,  7,  1,  3,  7, -1, -1, -1, -1, -1, -1, -1},
  {10,  7,  6,  1,  7, 10,  1,  8,  7,  1,  0,  8, -1, -1, -1, -1},
  { 0,  3,  7,  0,  7, 10,  0, 10,  9,  6, 10,  7, -1, -1, -1, -1},
  { 7,  6, 10,  7, 10,  8,  8, 10,  9, -1, -1, -1, -1, -1, -1, -1},
  { 6,  8,  4, 11,  8,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  6, 11,  3,  0,  6,  0,  4,  6, -1, -1, -1, -1, -1, -1, -1},
  { 8,  6, 11,  8,  4,  6,  9,  0,  1, -1, -1, -1, -1, -1, -1, -1},
  { 9,  4,  6,  9,  6,  3,  9,  3,  1, 11,  3,  6, -1, -1, -1, -1},
  { 6,  8,  4,  6, 11,  8,  2, 10,  1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10,  3,  0, 11,  0,  6, 11,  0,  4,  6, -1, -1, -1, -1},
  { 4, 11,  8,  4,  6, 11,  0,  2,  9,  2, 10,  9, -1, -1, -1, -1},
  {10,  9,  3, 10,  3,  2,  9,  4,  3, 11,  3,  6,  4,  6,  3, -1},
  { 8,  2,  3,  8,  4,  2,  4,  6,  2, -1, -1, -1, -1, -1, -1, -1},
  { 0,  4,  2,  4,  6,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  9,  0,  2,  3,  4,  2,  4,  6,  4,  3,  8, -1, -1, -1, -1},
  { 1,  9,  4,  1,  4,  2,  2,  4,  6, -1, -1, -1, -1, -1, -1, -1},
  { 8,  1,  3,  8,  6,  1,  8,  4,  6,  6, 10,  1, -1, -1, -1, -1},
  {10,  1,  0, 10,  0,  6,  6,  0,  4, -1, -1, -1, -1, -1, -1, -1},
  { 4,  6,  3,  4,  3,  8,  6, 10,  3,  0,  3,  9, 10,  9,  3, -1},
  {10,  9,  4,  6, 10,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  9,  5,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3,  4,  9,  5, 11,  7,  6, -1, -1, -1, -1, -1, -1, -1},
  { 5,  0,  1,  5,  4,  0,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1},
  {11,  7,  6,  8,  3,  4,  3,  5,  4,  3,  1,  5, -1, -1, -1, -1},
  { 9,  5,  4, 10,  1,  2,  7,  6, 11, -1, -1, -1, -1, -1, -1, -1},
  { 6, 11,  7,  1,  2, 10,  0,  8,  3,  4,  9,  5, -1, -1, -1, -1},
  { 7,  6, 11,  5,  4, 10,  4,  2, 10,  4,  0,  2, -1, -1, -1, -1},
  { 3,  4,  8,  3,  5,  4,  3,  2,  5, 10,  5,  2, 11,  7,  6, -1},
  { 7,  2,  3,  7,  6,  2,  5,  4,  9, -1, -1, -1, -1, -1, -1, -1},
  { 9,  5,  4,  0,  8,  6,  0,  6,  2,  6,  8,  7, -1, -1, -1, -1},
  { 3,  6,  2,  3,  7,  6,  1,  5,  0,  5,  4,  0, -1, -1, -1, -1},
  { 6,  2,  8,  6,  8,  7,  2,  1,  8,  4,  8,  5,  1,  5,  8, -1},
  { 9,  5,  4, 10,  1,  6,  1,  7,  6,  1,  3,  7, -1, -1, -1, -1},
  { 1,  6, 10,  1,  7,  6,  1,  0,  7,  8,  7,  0,  9,  5,  4, -1},
  { 4,  0, 10,  4, 10,  5,  0,  3, 10,  6, 10,  7,  3,  7, 10, -1},
  { 7,  6, 10,  7, 10,  8,  5,  4, 10,  4,  8, 10, -1, -1, -1, -1},
  { 6,  9,  5,  6, 11,  9, 11,  8,  9, -1, -1, -1, -1, -1, -1, -1},
  { 3,  6, 11,  0,  6,  3,  0,  5,  6,  0,  9,  5, -1, -1, -1, -1},
  { 0, 11,  8,  0,  5, 11,  0,  1,  5,  5,  6, 11, -1, -1, -1, -1},
  { 6, 11,  3,  6,  3,  5,  5,  3,  1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 10,  9,  5, 11,  9, 11,  8, 11,  5,  6, -1, -1, -1, -1},
  { 0, 11,  3,  0,  6, 11,  0,  9,  6,  5,  6,  9,  1,  2, 10, -1},
  {11,  8,  5, 11,  5,  6,  8,  0,  5, 10,  5,  2,  0,  2,  5, -1},
  { 6, 11,  3,  6,  3,  5,  2, 10,  3, 10,  5,  3, -1, -1, -1, -1},
  { 5,  8,  9,  5,  2,  8,  5,  6,  2,  3,  8,  2, -1, -1, -1, -1},
  { 9,  5,  6,  9,  6,  0,  0,  6,  2, -1, -1, -1, -1, -1, -1, -1},
  { 1,  5,  8,  1,  8,  0,  5,  6,  8,  3,  8,  2,  6,  2,  8, -1},
  { 1,  5,  6,  2,  1,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  3,  6,  1,  6, 10,  3,  8,  6,  5,  6,  9,  8,  9,  6, -1},
  {10,  1,  0, 10,  0,  6,  9,  5,  0,  5,  6,  0, -1, -1, -1, -1},
  { 0,  3,  8,  5,  6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {10,  5,  6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {11,  5, 10,  7,  5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {11,  5, 10, 11,  7,  5,  8,  3,  0, -1, -1, -1, -1, -1, -1, -1},
  { 5, 11,  7,  5, 10, 11,  1,  9,  0, -1, -1, -1, -1, -1, -1, -1},
  {10,  7,  5, 10, 11,  7,  9,  8,  1,  8,  3,  1, -1, -1, -1, -1},
  {11,  1,  2, 11,  7,  1,  7,  5,  1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3,  1,  2,  7,  1,  7,  5,  7,  2, 11, -1, -1, -1, -1},
  { 9,  7,  5,  9,  2,  7,  9,  0,  2,  2, 11,  7, -1, -1, -1, -1},
  { 7,  5,  2,  7,  2, 11,  5,  9,  2,  3,  2,  8,  9,  8,  2, -1},
  { 2,  5, 10,  2,  3,  5,  3,  7,  5, -1, -1, -1, -1, -1, -1, -1},
  { 8,  2,  0,  8,  5,  2,  8,  7,  5, 10,  2,  5, -1, -1, -1, -1},
  { 9,  0,  1,  5, 10,  3,  5,  3,  7,  3, 10,  2, -1, -1, -1, -1},
  { 9,  8,  2,  9,  2,  1,  8,  7,  2, 10,  2,  5,  7,  5,  2, -1},
  { 1,  3,  5,  3,  7,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  7,  0,  7,  1,  1,  7,  5, -1, -1, -1, -1, -1, -1, -1},
  { 9,  0,  3,  9,  3,  5,  5,  3,  7, -1, -1, -1, -1, -1, -1, -1},
  { 9,  8,  7,  5,  9,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 5,  8,  4,  5, 10,  8, 10, 11,  8, -1, -1, -1, -1, -1, -1, -1},
  { 5,  0,  4,  5, 11,  0,  5, 10, 11, 11,  3,  0, -1, -1, -1, -1},
  { 0,  1,  9,  8,  4, 10,  8, 10, 11, 10,  4,  5, -1, -1, -1, -1},
  {10, 11,  4, 10,  4,  5, 11,  3,  4,  9,  4,  1,  3,  1,  4, -1},
  { 2,  5,  1,  2,  8,  5,  2, 11,  8,  4,  5,  8, -1, -1, -1, -1},
  { 0,  4, 11,  0, 11,  3,  4,  5, 11,  2, 11,  1,  5,  1, 11, -1},
  { 0,  2,  5,  0,  5,  9,  2, 11,  5,  4,  5,  8, 11,  8,  5, -1},
  { 9,  4,  5,  2, 11,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 2,  5, 10,  3,  5,  2,  3,  4,  5,  3,  8,  4, -1, -1, -1, -1},
  { 5, 10,  2,  5,  2,  4,  4,  2,  0, -1, -1, -1, -1, -1, -1, -1},
  { 3, 10,  2,  3,  5, 10,  3,  8,  5,  4,  5,  8,  0,  1,  9, -1},
  { 5, 10,  2,  5,  2,  4,  1,  9,  2,  9,  4,  2, -1, -1, -1, -1},
  { 8,  4,  5,  8,  5,  3,  3,  5,  1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  4,  5,  1,  0,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 8,  4,  5,  8,  5,  3,  9,  0,  5,  0,  3,  5, -1, -1, -1, -1},
  { 9,  4,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4, 11,  7,  4,  9, 11,  9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
  { 0,  8,  3,  4,  9,  7,  9, 11,  7,  9, 10, 11, -1, -1, -1, -1},
  { 1, 10, 11,  1, 11,  4,  1,  4,  0,  7,  4, 11, -1, -1, -1, -1},
  { 3,  1,  4,  3,  4,  8,  1, 10,  4,  7,  4, 11, 10, 11,  4, -1},
  { 4, 11,  7,  9, 11,  4,  9,  2, 11,  9,  1,  2, -1, -1, -1, -1},
  { 9,  7,  4,  9, 11,  7,  9,  1, 11,  2, 11,  1,  0,  8,  3, -1},
  {11,  7,  4, 11,  4,  2,  2,  4,  0, -1, -1, -1, -1, -1, -1, -1},
  {11,  7,  4, 11,  4,  2,  8,  3,  4,  3,  2,  4, -1, -1, -1, -1},
  { 2,  9, 10,  2,  7,  9,  2,  3,  7,  7,  4,  9, -1, -1, -1, -1},
  { 9, 10,  7,  9,  7,  4, 10,  2,  7,  8,  7,  0,  2,  0,  7, -1},
  { 3,  7, 10,  3, 10,  2,  7,  4, 10,  1, 10,  0,  4,  0, 10, -1},
  { 1, 10,  2,  8,  7,  4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  9,  1,  4,  1,  7,  7,  1,  3, -1, -1, -1, -1, -1, -1, -1},
  { 4,  9,  1,  4,  1,  7,  0,  8,  1,  8,  7,  1, -1, -1, -1, -1},
  { 4,  0,  3,  7,  4,  3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 4,  8,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 9, 10,  8, 10, 11,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  0,  9,  3,  9, 11, 11,  9, 10, -1, -1, -1, -1, -1, -1, -1},
  { 0,  1, 10,  0, 10,  8,  8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
  { 3,  1, 10, 11,  3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  2, 11,  1, 11,  9,  9, 11,  8, -1, -1, -1, -1, -1, -1, -1},
  { 3,  0,  9,  3,  9, 11,  1,  2,  9,  2, 11,  9, -1, -1, -1, -1},
  { 0,  2, 11,  8,  0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 3,  2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 2,  3,  8,  2,  8, 10, 10,  8,  9, -1, -1, -1, -1, -1, -1, -1},
  { 9, 10,  2,  0,  9,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 2,  3,  8,  2,  8, 10,  0,  1,  8,  1, 10,  8, -1, -1, -1, -1},
  { 1, 10,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 1,  3,  8,  9,  1,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  9,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  { 0,  3,  8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::poly_Mesh>
  mobius::poly_MarchingCubes::PolygonizeVoxel(const t_xyz&                    P0,
                                              const t_xyz&                    P7,
                                              const t_ptr<poly_ImplicitFunc>& func,
                                              const double                    isoValue)
{
  t_ptr<poly_Mesh> result = new poly_Mesh;

  t_xyz P1( P7.X(), P0.Y(), P0.Z() );
  t_xyz P2( P0.X(), P7.Y(), P0.Z() );
  t_xyz P3( P7.X(), P7.Y(), P0.Z() );
  t_xyz P4( P0.X(), P0.Y(), P7.Z() );
  t_xyz P5( P7.X(), P0.Y(), P7.Z() );
  t_xyz P6( P0.X(), P7.Y(), P7.Z() );

  // Initialize scalars at voxel corners.
  double voxelScalars[2][2][2];
  //
  voxelScalars[0][0][0] = func->Eval( P0.X(), P0.Y(), P0.Z() ) - isoValue;
  voxelScalars[1][0][0] = func->Eval( P1.X(), P1.Y(), P1.Z() ) - isoValue;
  voxelScalars[0][1][0] = func->Eval( P2.X(), P2.Y(), P2.Z() ) - isoValue;
  voxelScalars[1][1][0] = func->Eval( P3.X(), P3.Y(), P3.Z() ) - isoValue;
  voxelScalars[0][0][1] = func->Eval( P4.X(), P4.Y(), P4.Z() ) - isoValue;
  voxelScalars[1][0][1] = func->Eval( P5.X(), P5.Y(), P5.Z() ) - isoValue;
  voxelScalars[0][1][1] = func->Eval( P6.X(), P6.Y(), P6.Z() ) - isoValue;
  voxelScalars[1][1][1] = func->Eval( P7.X(), P7.Y(), P7.Z() ) - isoValue;

  /* Determine the cube index for subsequent lookup */
  const int cubeIndex = GetCubeIndex(voxelScalars);

  /* Process intersection */
  if ( aiCubeEdgeFlags[cubeIndex] == 0 )
    return result;

  /* Find vertices whether the surface intersects the cube */
  t_xyz vertices[12];
  //
  if ( aiCubeEdgeFlags[cubeIndex] & 1 )
  {
    vertices[0] = InterpVertex( P0, P1,
                                voxelScalars[0][0][0],
                                voxelScalars[1][0][0] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 2 )
  {
    vertices[1] = InterpVertex( P1, P3,
                                voxelScalars[1][0][0],
                                voxelScalars[1][1][0] );
    }

  if ( aiCubeEdgeFlags[cubeIndex] & 4 )
  {
    vertices[2] = InterpVertex( P3, P2,
                                voxelScalars[1][1][0],
                                voxelScalars[0][1][0] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 8 )
  {
    vertices[3] = InterpVertex( P2, P0,
                                voxelScalars[0][1][0],
                                voxelScalars[0][0][0] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 16 )
  {
    vertices[4] = InterpVertex( P4, P5,
                                voxelScalars[0][0][1],
                                voxelScalars[1][0][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 32 )
  {
    vertices[5] = InterpVertex( P5, P7,
                                voxelScalars[1][0][1],
                                voxelScalars[1][1][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 64 )
  {
    vertices[6] = InterpVertex( P7, P6,
                                voxelScalars[1][1][1],
                                voxelScalars[0][1][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 128 )
  {
    vertices[7] = InterpVertex( P6, P4,
                                voxelScalars[0][1][1],
                                voxelScalars[0][0][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 256 )
  {
    vertices[8] = InterpVertex( P0, P4,
                                voxelScalars[0][0][0],
                                voxelScalars[0][0][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 512 )
  {
    vertices[9] = InterpVertex( P1, P5,
                                voxelScalars[1][0][0],
                                voxelScalars[1][0][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 1024 )
  {
    vertices[10] = InterpVertex( P3, P7,
                                 voxelScalars[1][1][0],
                                 voxelScalars[1][1][1] );
  }

  if ( aiCubeEdgeFlags[cubeIndex] & 2048 )
  {
    vertices[11] = InterpVertex( P2, P6,
                                 voxelScalars[0][1][0],
                                 voxelScalars[0][1][1] );
  }

  /* Build triangles */
  const int* triIndices = a2iTriangleConnectionTable[cubeIndex];
  for ( int tt = 0; triIndices[tt] != -1; tt += 3 )
  {
    std::vector<int> tri = {0, 0, 0, 0};
    //
    for ( int k = 0; k < 3; ++k )
    {
      const int    triId  = triIndices[tt + 2 - k];
      const t_xyz& vertex = vertices[triId];

      // Add vertex.
      poly_VertexHandle hv = result->AddVertex(vertex);

      tri[k] = hv.GetIdx();
    }

    // Add triangle.
    result->AddTriangle( poly_VertexHandle(tri[0]),
                         poly_VertexHandle(tri[1]),
                         poly_VertexHandle(tri[2]) );
  }

  return result;
}

//-----------------------------------------------------------------------------

int mobius::poly_MarchingCubes::GetCubeIndex(double voxelScalars[2][2][2])
{
  /* Determine the cube index for subsequent lookup */
  int cubeIndex = 0;
  //
  if ( voxelScalars[0][0][0] < 0.0 )
    cubeIndex |= 1;   // | <0000 0001>
  //
  if ( voxelScalars[1][0][0] < 0.0 )
    cubeIndex |= 2;   // | <0000 0010>
  //
  if ( voxelScalars[1][1][0] < 0.0 )
    cubeIndex |= 4;   // | <0000 0100>
  //
  if ( voxelScalars[0][1][0] < 0.0 )
    cubeIndex |= 8;   // | <0000 1000>
  //
  if ( voxelScalars[0][0][1] < 0.0 )
    cubeIndex |= 16;  // | <0001 0000>
  //
  if ( voxelScalars[1][0][1] < 0.0 )
    cubeIndex |= 32;  // | <0010 0000>
  //
  if ( voxelScalars[1][1][1] < 0.0 )
    cubeIndex |= 64;  // | <0100 0000>
  //
  if ( voxelScalars[0][1][1] < 0.0 )
    cubeIndex |= 128; // | <1000 0000>

  return cubeIndex;
}

//-----------------------------------------------------------------------------

mobius::t_xyz mobius::poly_MarchingCubes::GetVoxelCorner(const t_xyz& origin,
                                                         const double dx,
                                                         const double dy,
                                                         const double dz,
                                                         const int    nx,
                                                         const int    ny,
                                                         const int    nz)
{
  const double Px = origin.X() + dx*nx;
  const double Py = origin.Y() + dy*ny;
  const double Pz = origin.Z() + dz*nz;

  return t_xyz(Px, Py, Pz);
}

//-----------------------------------------------------------------------------

mobius::t_xyz mobius::poly_MarchingCubes::InterpVertex(const t_xyz& point1,
                                                       const t_xyz& point2,
                                                       const double scalar1,
                                                       const double scalar2)
{
  return point1 - (point2 - point1) * ( scalar1/(scalar2 - scalar1) );
}

//-----------------------------------------------------------------------------

mobius::poly_MarchingCubes::poly_MarchingCubes(const t_ptr<poly_ImplicitFunc>& func,
                                               const int                       numSlices,
                                               core_ProgressEntry              progress,
                                               core_PlotterEntry               plotter)
: poly_GridTessellator(func->GetDomainMin(),
                       func->GetDomainMax(),
                       numSlices,
                       progress,
                       plotter),
  m_func(func)
{}

//-----------------------------------------------------------------------------

bool mobius::poly_MarchingCubes::perform(const double isoValue)
{
  t_vertexMap vertexMap;

  // Iterate grid.
  for ( int nz = 0; nz < m_iNumSlicesZ; ++nz )
  {
    for ( int ny = 0; ny < m_iNumSlicesY; ++ny )
    {
      for ( int nx = 0; nx < m_iNumSlicesX; ++nx )
      {

        /* Compute scalars in a single voxel (8 values) */
        double voxelScalars[2][2][2];
        for ( int nvz = 0; nvz < 2; ++nvz )
        {
          for ( int nvy = 0; nvy < 2; ++nvy )
          {
            for ( int nvx = 0; nvx < 2; ++nvx )
            {
              const t_xyz P = GetVoxelCorner(m_Pmin,
                                             m_fGrainX,
                                             m_fGrainY,
                                             m_fGrainZ,
                                             nx + nvx,
                                             ny + nvy,
                                             nz + nvz);

              const double f = m_func->Eval( P.X(), P.Y(), P.Z() ) - isoValue;

              voxelScalars[nvx][nvy][nvz] = f;
            }
          }
        }

        /* Determine the cube index for subsequent lookup */
        const int cubeIndex = GetCubeIndex(voxelScalars);

        /* Process intersection */
        if ( aiCubeEdgeFlags[cubeIndex] != 0 )
        {
          /* Find vertices whether the surface intersects the cube */
          t_xyz vertices[12];
          //
          if ( aiCubeEdgeFlags[cubeIndex] & 1 )
          {
            vertices[0] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 0),
                                        voxelScalars[0][0][0],
                                        voxelScalars[1][0][0] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 2 )
          {
            vertices[1] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 1, nz + 0),
                                        voxelScalars[1][0][0],
                                        voxelScalars[1][1][0] );
            }

          if ( aiCubeEdgeFlags[cubeIndex] & 4 )
          {
            vertices[2] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 1, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 1, nz + 0),
                                        voxelScalars[1][1][0],
                                        voxelScalars[0][1][0] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 8 )
          {
            vertices[3] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 1, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 0),
                                        voxelScalars[0][1][0],
                                        voxelScalars[0][0][0] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 16 )
          {
            vertices[4] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 1),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 1),
                                        voxelScalars[0][0][1],
                                        voxelScalars[1][0][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 32 )
          {
            vertices[5] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 1),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 1, nz + 1),
                                        voxelScalars[1][0][1],
                                        voxelScalars[1][1][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 64 )
          {
            vertices[6] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 1, nz + 1),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 1, nz + 1),
                                        voxelScalars[1][1][1],
                                        voxelScalars[0][1][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 128 )
          {
            vertices[7] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 1, nz + 1),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 1),
                                        voxelScalars[0][1][1],
                                        voxelScalars[0][0][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 256 )
          {
            vertices[8] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 0, ny + 0, nz + 1),
                                        voxelScalars[0][0][0],
                                        voxelScalars[0][0][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 512 )
          {
            vertices[9] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 0),
                                        GetVoxelCorner(m_Pmin,
                                                       m_fGrainX,
                                                       m_fGrainY,
                                                       m_fGrainZ,
                                                       nx + 1, ny + 0, nz + 1),
                                        voxelScalars[1][0][0],
                                        voxelScalars[1][0][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 1024 )
          {
            vertices[10] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                        m_fGrainX,
                                                        m_fGrainY,
                                                        m_fGrainZ,
                                                        nx + 1, ny + 1, nz + 0),
                                         GetVoxelCorner(m_Pmin,
                                                        m_fGrainX,
                                                        m_fGrainY,
                                                        m_fGrainZ,
                                                        nx + 1, ny + 1, nz + 1),
                                         voxelScalars[1][1][0],
                                         voxelScalars[1][1][1] );
          }

          if ( aiCubeEdgeFlags[cubeIndex] & 2048 )
          {
            vertices[11] = InterpVertex( GetVoxelCorner(m_Pmin,
                                                        m_fGrainX,
                                                        m_fGrainY,
                                                        m_fGrainZ,
                                                        nx + 0, ny + 1, nz + 0),
                                         GetVoxelCorner(m_Pmin,
                                                        m_fGrainX,
                                                        m_fGrainY,
                                                        m_fGrainZ,
                                                        nx + 0, ny + 1, nz + 1),
                                         voxelScalars[0][1][0],
                                         voxelScalars[0][1][1] );
          }

          /* Build triangles */
          const int* triIndices = a2iTriangleConnectionTable[cubeIndex];
          for ( int tt = 0; triIndices[tt] != -1; tt += 3 )
          {
            std::vector<int> tri = {0, 0, 0, 0};
            //
            for ( int k = 0; k < 3; ++k )
            {
              const int    triId  = triIndices[tt + 2 - k];
              const t_xyz& vertex = vertices[triId];

              auto vertexIt = vertexMap.lower_bound(vertex); // search for the vertex.
              if ( vertexIt == vertexMap.end() || vertexMap.key_comp()(vertex, vertexIt->first) )
              {
                // Add vertex.
                poly_VertexHandle hv = m_result->AddVertex(vertex);

                vertexIt = vertexMap.insert( vertexIt,
                                             std::make_pair( vertex, hv.GetIdx() ) );
              }

              tri[k] = vertexIt->second;
            }

            // Add triangle.
            m_result->AddTriangle( poly_VertexHandle(tri[0]),
                                   poly_VertexHandle(tri[1]),
                                   poly_VertexHandle(tri[2]) );
          }
        }

      }
    }
  }

  return true;
}
