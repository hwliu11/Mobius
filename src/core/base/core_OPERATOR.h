//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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

#ifndef core_OPERATOR_HeaderFile
#define core_OPERATOR_HeaderFile

// Core includes
#include <mobius/core_IAlgorithm.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Interface for modeling operators. By operator we understand something
//! intuitively close to a symbol that can be used to denote a particular
//! operation. The difference between operators, operations, algorithms,
//! functions, and procedures is something not very clear. Here is the
//! reasoning which we found very close to our own intuition
//! [math.stackexchange.com]:
//!
//! Function: a set of pairs constructed from two sets called the domain and
//!           codomain (target set). A function can be seen as an abstract
//!           (meaning "imagined" here) machine that consumes values
//!           from the domain and produces values from the codomain.
//!
//! Operator: usually in higher math and physics contexts is a function whose
//!           domain includes other functions. Computer scientists call these
//!           "higher order functions."
//!
//! Algorithm: a function with a well defined abstract implementation. Rather
//!            than just being a set theoretic invention (i.e., a set of pairs
//!            as functions are generally defined), algorithms are functions
//!            whose internal operations/calculation has a story. In the earlier
//!            analogy from functions, the abstract machines of algorithms can
//!            actually be picked apart and described. It is possible to talk
//!            about how long the machine runs, what subcomponents it has, how
//!            much space it uses etc... An algorithm is a "how" of a function.
//!
//! Procedure: a super set of algorithms. Procedures do not have a connotation
//!            of mapping inputs to outputs, although they can if you want.
//!            A close notion to procedure is a "computational scheme," i.e.,
//!            the algorithms combined to a super-algorithm.
//!
//! Operation: a substitute for the word "function: usually used when you want
//!            to treat the function as a single atomic object. Ex: order of
//!            operations (here functions are the little things, and we are
//!            looking at a larger idea of how to evaluate them, what order, etc.).
//!            Another example: quicksort takes O(n log n) floating point operations
//!            on randomized inputs. Here operation is short hand for: addition,
//!            subtraction, division, multiplication, comparison. But those
//!            functions and their inputs (integers) really are not the main
//!            point. The main point is thatwe want to count how many times we
//!            do them, so we call them operations to make this context clear.
//!
//! The operator for us a like a symbol, e.g., `P1 P2 P3 s` is a modeling
//! chain composed of operators `P3`, then `P2`, then `P1` applied to the
//! abstract shape `s`. An operator can be seen as a function which maps
//! one abstract shape with some representation scheme to another abstract
//! shape possibly having a different representation scheme (e.g., voxels
//! to b-rep mesh). An operator is also an algorithm, i.e., it is a sequence
//! of operations to produce the result.
//!
//! In Mobius, we have OBJECTs and OPERATORs working on them. Another way of
//! seeing it is to say the OPERATOR is a modeling algorithm in Mobius.
class core_OPERATOR : public core_IAlgorithm
{
protected:

  mobiusCore_EXPORT
    core_OPERATOR(core_ProgressEntry progress,
                  core_PlotterEntry  plotter);

private:

  core_OPERATOR() : core_IAlgorithm() {} //!< Default ctor.

};

};

#endif
