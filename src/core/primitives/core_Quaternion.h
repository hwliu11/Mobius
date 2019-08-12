//-----------------------------------------------------------------------------
// Created on: 15 January 2014
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

#ifndef core_Quaternion_HeaderFile
#define core_Quaternion_HeaderFile

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Quaternion implementation for Mobius. The following description represents
//! a short memo based on "Byung-Uk Lee, Unit Quaternion Representation of
//! Rotation" paper. This memo can take you through the most important aspects
//! of quaternions which might be necessary in order to get started. If you
//! are already familiar with a concept of quaternion, you do not need to
//! read the following text.
//!
//! [MEMO]
//!
//! The usual way to work with 3D rotations in Mobius implies usage of
//! quaternion concept. Generally speaking, quaternion is a simple structure
//! which can be used to express rotation by the given angle around the
//! given axis.
//!
//! It is necessary to keep in mind that very often in engineering
//! applications the axes have physical notation, rather than geometrical one.
//! Physical notation of axis (vector) presumes that the axis is imposed to
//! some position of interest. E.g. forces are physical vectors as they
//! essentially contain information about the points they are acting on.
//! Also it is quite usual to use COG (center of gravity) as a reference point
//! for rotations. All this is mentioned to say that quaternions imply
//! geometrical notation of vectors. I.e. when describing rotation with
//! quaternion, the only thing which does matter is the relative disposition
//! of the rotated point and axis. Being more practical, it is correct to
//! say, that for any point the corresponding axis of rotation is always
//! located in the local origin of the object's space. Just keep this in
//! mind, and once you want to rotate your object around some physical axis
//! (e.g. around COG), recall that quaternions do not give you such a
//! possibility directly. The usual way to rotate something around non-trivial
//! reference point is first to translate this point to the origin and
//! only then to apply a rotation. It is obvious that in this two-stage
//! transformation quaternions come into play in the second stage and do not
//! do the entire job.
//!
//! Now we give a short overview for quaternion concept.
//!
//! Quaternions were defined by Hamilton in 19-th century. Strictly speaking,
//! quaternion is a generalization of the notion of complex number to
//! four-dimensional space. Its mathematical equation is the following:
//!
//!    q = q0 + i*qx + j*qy + k*qz;
//!
//! Here i, j, k are imaginary components having the following properties:
//!
//!    i^2 = j^2 = k^2 = -1;
//!    i*j =  k; j*k =  i; k*i =  j;
//!    j*i = -k; k*j = -i; i*k = -j;
//!
//! Another common way to represent quaternions is the following:
//!
//!    q = (q_real, q_imag);
//!
//! Here q_real is the so called pure real quaternion containing just one
//! floating-point number. The q_imag part is three-dimensional vector
//! called a pure imaginary quaternion. It is obvious that any vector in 3D
//! can be thought of as a pure imaginary quaternion.
//!
//! Following common algebraic rules and taking into account the properties
//! of imaginary units (i, j, k), its is easy to check that multiplication
//! of quaternions is not commutative.
//!
//! The conjugate of q is defined similarly to that of complex numbers --
//! by inverting signs of imaginary units. The conjugate quaternion to q is
//! denoted as (q*).
//!
//! Hereinafter we enumerate the useful properties of quaternions:
//!
//!----------------------------------------------------------------------------
//! 0. Algebraic product: p*q
//!----------------------------------------------------------------------------
//!    p*q =     (p0*q0 - px*qx - py*qy - pz*qz)
//!          + i*(p0*qx + px*q0 + py*qz - pz*qy)
//!          + j*(p0*qy - px*qz + py*q0 + pz*qx)
//!          + k*(p0*qz + px*qy - py*qx + pz*q0);
//!
//!----------------------------------------------------------------------------
//! 1. Conjugation of algebraic product: (p*q)*
//!----------------------------------------------------------------------------
//!    (p*q)* = (q*)*(p*);
//!
//!----------------------------------------------------------------------------
//! 2. Dot product: <p,q>
//!----------------------------------------------------------------------------
//!    <p,q> = p0*q0 + px*qx + py*qy + pz*qz = (p*(q*) + q*(p*))/2;
//!
//!----------------------------------------------------------------------------
//! 3. Inversion: q^(-1)
//!----------------------------------------------------------------------------
//!    q^(-1) = (q*) * (1/<q,q>);
//!
//!----------------------------------------------------------------------------
//! 4. Cross product: [p,q]
//!----------------------------------------------------------------------------
//!    [p,q] = (p*q - q*p)/2;
//!
//! Having all these formulae enumerated, we are going to demonstrate the
//! property which is the most important for us when dealing with rotations:
//!
//!----------------------------------------------------------------------------
//! 5. Rotation formula
//!----------------------------------------------------------------------------
//!    t' = q*t*(q*);
//!
//! Here t is some input vector and t' is a result of some rotation. Therefore,
//! we state that quaternions can be used according to [5] in order to perform
//! a rotation operation. It is easy to see that if |q| = 1, then the
//! mentioned formula can be rewritten according to [3]:
//!
//!    t' = q*t*q^(-1);
//!
//! The only thing we need to assure is that [5] is really a description of
//! rotation.
//!
//!----------------------------------------------------------------------------
//! A. Pure imaginarity
//!----------------------------------------------------------------------------
//!
//! First of all, we need to assure that t' is still a vector in 3D space, i.e.
//! it is a pure imaginary quaternion. Here we need to notice that for pure
//! imaginary quaternions the following statement is true (obvious by
//! definition of conjugate quaternion):
//!
//!    q + (q*) = 0;
//!
//! Then we check:
//!
//!    t' + (t'*) = q*t*(q*) + (q*t*(q*))* = q*t*(q*) + q*(t*)*(q*) =
//!               = q*(t + (t*))*(q*) = 0;
//!
//! The latter term is equal to zero as (t + (t*)) obviously vanishes. Thus
//! we have shown that [5] preserves the property of pure imaginarity and so
//! 3D vectors are mapped again to 3D vectors -- t' is pure imaginary.
//!
//!----------------------------------------------------------------------------
//! B. Invariance of dot product
//!----------------------------------------------------------------------------
//!
//! Another property of rotation is invariance of dot products between the
//! affected vectors. In order to bring it out, we consider the dot product
//! of transformed pure imaginary quaternions:
//!
//!    <s',t'> = <q*s*(q*),q*t*(q*)>;
//!
//! According to [2], we have:
//!
//!    <s',t'> = ( (s')*(t'*) + (t')*(s'*) )/2 =
//!            = ( (q*s*(q*))*(q*t*(q*))* + (q*t*(q*))*(q*s*(q*))* )/2 =
//!            = ( q*s*(q*)*q*(t*)*(q*) + q*t*(q*)*q*(s*)*(q*) )/2;
//!
//! Taking into account that q is pure imaginary (q0 = 0) and |q| = 1, we can
//! easily show (according to [0]) that the algebraic product (q*)*q is equal
//! to 1. Then we obtain:
//!
//!    <s',t'> = ( q*s*(t*)*(q*) + q*t*(s*)*(q*) )/2 =
//!            = q * {(s*(t*) + t*s(*))/2} * (q*) =
//!            = q*<s,t>*(q*);
//!
//! Here we can notice that <s,t> is a real number as a result of dot product.
//! Obviously, the product of a real number with a quaternion is commutative.
//! Therefore, we have:
//!
//!    <s',t'> = q*<s,t>*(q*) = <s,t>*q*(q*) = <s,t>;
//!
//! Q.E.D.
//!
//! NOTE 1: as a special case we have <t',t'> = <t,t>, which means that
//!         the norm of a vector is invariant regarding to [5]. This, inter
//!         alia, means that [5] cannot be an enlargement and is not connected
//!         to translation transformations anyhow.
//!
//! Having proved that dot product is invariant, we reduced the set of
//! suitable transformations which can be expressed by [5] to rotations and
//! reflections. The reflection operation switches right-handed reference
//! box to left-handed one or vice versa, which results in sign change of
//! the cross product. Therefore, the last thing to shown is invariance of
//! the cross product against [5].
//!
//!----------------------------------------------------------------------------
//! C. Cross product property
//!----------------------------------------------------------------------------
//!
//!    [s',t'] = (s'*t' - t'*s')/2 =
//!            = (q*s*(q*)*q*t*(q*) - q*t*(q*)*q*s*(q*))/2 =
//!            = (q*s*t*(q*) - q*t*s*(q*))/2 =
//!            = q * {(s*t - t*s)/2} * (q*) =
//!            = q*[s,t]*(q*);
//!
//! From here we see that image of the cross product of original vectors (rhs)
//! is equal to the cross product of their images (lhs). This is completely a
//! rotational Feng Shui which is not a property of reflections. Therefore,
//! we can conclude that [5] specifies rotation.
//!
//! Q.E.D.
//!----------------------------------------------------------------------------
//! At this point we already know that quaternion can be used to define "some"
//! rotation. In order to apply rotation to some vector, we have to use
//! formula [5]. However, we still know nothing about the properties of such
//! rotation. Actually, what we need to know is:
//!
//! - what is the axis of rotation?
//! - what is the angle of rotation?
//!
//! Obviously, talking about some theoretical rotation without knowing its
//! properties is just impractical. What we are going to do now, is to pick up
//! some axis {w} (omega), angle {a} (alpha) and to write down the formula
//! of the corresponding rotational transformation just from trigonometric
//! consideration. Then we will collate [5] with the obtained formula, and
//! this way we will see, what is the role of {q} in such rotation.
//!
//! Let us take a look on the following illustration:
//!
//!       w ^
//!         |
//!         |
//!       C +----------------+ P
//!        / \
//!       /====\
//!      /  a    \
//!     /   |      \
//!    /    |       + T'
//! T /     |      /
//!  +      |     /
//!   \     |    /
//!    \    |   /
//!   t \   |  / t'
//!      \  | /
//!       \ |/
//!        \|/
//!       O +
//!         |
//!
//! Here we see vector {t} pointing to position {T} and being rotated around
//! unit axis {w} by angle {a}. The result of rotation is point {T'}. The
//! center of the chosen coordinate system is denoted as {O}. Angle {TCP} is
//! 90 degrees.
//!
//! In order to benefit from [5], we need to find formulation for {t'}.
//! Obviously we have:
//!
//!    t' = OC + CT';
//!
//! Here all values have vectorial meaning, so directions does matter. On the
//! other hand we have:
//!
//!    OC = <t,w>*w;
//!
//! Therefore, OC is found as projection of {t} to the unit direction {w}.
//! Next we need to find an expression for {CT'}.
//!
//!    CT = t - OC = t - <t,w>*w;
//!    CP = [w,CT] = [w,t] - [w,<t,w>*w] = [w,t];
//!
//! Finally we have to note that the sought-for {CT'} is a linear combination
//! of {CT} and {CP}, i.e:
//!
//!    CT' = L1(a)*CT + L2(a)*CP;
//!
//! Here {L1} and {L2} are angle-dependent coefficients. It is easy to see
//! that:
//!
//!    L1(a) = cos(a);
//!    L2(a) = sin(a);
//!
//! So, finally we have:
//!
//! [6] t' = OC + CT' = <t,w>*w + cos(a)*(t - <t,w>*w) + sin(a)*[w,t]
//!        = t*cos(a) + [w,t]*sin(a) + <t,w>*w*(1 - cos(a));
//!
//! Now let us introduce new quaternion {q} in the following form:
//!
//!    q = cos(a/2) + sin(a/2)*w;
//!    q* = cos(a/2) + sin(a/2)*(w*);
//!
//! Please note that {w} is pure imaginary quaternion corresponding to the
//! axis of rotation. I.e. it contains 3 components corresponding to imaginary
//! units (i, j, k).
//!
//! [7] q*t*(q*) = (cos(a/2) + sin(a/2)*w)*t*(cos(a/2) + sin(a/2)*(w*))
//!              = t*(cos(a/2))^2 + cos(a/2)*sin(a/2)*(w*t + t*(w*)) + w*t*(w*)*(sin(a/2))^2;
//!
//! Let us consider the following term:
//!
//!    2*<t,w>*w = (t*(w*) + w*(t*))*w = t*(w*)*w + w*(t*)*w = t + w*t*(w*);
//!
//! Therefore:
//!
//!    w*t*(w*) = 2*<t,w>*w - t;
//!
//! Then, returning to [7], we have:
//!
//!    q*t*(q*) = t*(cos(a/2))^2 + cos(a/2)*sin(a/2)*(w*t - t*w) + (2*<t,w>*w - t)*(sin(a/2))^2 =
//!             = t*((cos(a/2))^2 -(sin(a/2))^2) + 2*cos(a/2)*sin(a/2)*[w,t] + 2*(sin(a/2))^2*<t,w>*w =
//!             = t*cos(a) + [w,t]*sin(a) + (1 - cos(a))*<t,w>*w;
//!
//! And this is exactly [6]. Therefore, we have shown that rotation around
//! axis {w} is expressed by quaternion {q = cos(a/2) + sin(a/2)*w}.
//!
//! Q.E.D.
//!----------------------------------------------------------------------------
//! Conclusion: from transparent geometric consideration implying dot and
//! cross products we have switched to algebraic product {*} which was defined
//! in very unintuitive way in [0]. Conceptually, what we actually did is
//! introduced a completely new number set (called quaternions), gave these
//! numbers arithmetic rules and proved that formula [6] can be simplified
//! to [5] on these numbers.
//!
//! As a result we now have quite intuitive and compact way for representation
//! of rotations.
//!----------------------------------------------------------------------------
//! Even though quaternions are easy to manipulate, very often we might need
//! to switch to more "classic" matrix representation of rotation. Indeed,
//! as rotation is nothing but an isometric linear transformation, it is
//! necessary to be able to construct its matrix. This matrix is obtained
//! from simple unpacking of the general formula [5]:
//!
//!    t' = q*t*(q*) = ...;
//!
//! We take advantage of algebraic rules in order to unpack this formulation.
//! Please note, that here we consider {t} as pure imaginary quaternion in
//! order to perform all necessary arithmetic.
//!
//!    q = q0 + i*qx + j*qy + k*qz;
//!    q* = q0 - i*qx - j*qy - k*qz;
//!    t = i*tx + j*ty + k*tz;
//!
//! After applying formula [5], we obtain the following matrix:
//!
//!    |tx'| = | (q0^2 + qx^2 - qy^2 - qz^2) (-2*q0*qz + 2*qx*qy)        (2*q0*qy + 2*qx*qz)         | |tx|
//!    |ty'| = | (2*q0*qz + 2*qx*qy)         (q0^2 - qx^2 + qy^2 - qz^2) (-2*q0*qx + 2*qy*qz)        | |ty|
//!    |tz'| = | (-2*q0*qy + 2*qx*qz)        (2*q0*qx + 2*qy*qz)         (q0^2 - qx^2 - qy^2 + qz^2) | |tz|
class core_Quaternion
{
// Construction & destruction:
public:

  mobiusCore_EXPORT
    core_Quaternion();

  mobiusCore_EXPORT
    core_Quaternion(const double p0,
                    const double px,
                    const double py,
                    const double pz);

  mobiusCore_EXPORT
    core_Quaternion(const core_XYZ& axis,
                    const double    angle);

  mobiusCore_EXPORT
    core_Quaternion(const core_XYZ& Qn_im);

  mobiusCore_EXPORT
    core_Quaternion(const core_Quaternion& Qn);

  mobiusCore_EXPORT virtual
    ~core_Quaternion();

public:

  mobiusCore_EXPORT void
    SetRotation(const core_XYZ& axis,
                const double    angle);

  mobiusCore_EXPORT void
    Matrix3x3(double (&mx)[3][3]) const;

  mobiusCore_EXPORT double
    Q0() const;

  mobiusCore_EXPORT double
    Qx() const;

  mobiusCore_EXPORT double
    Qy() const;

  mobiusCore_EXPORT double
    Qz() const;

  mobiusCore_EXPORT bool
    AsVector(core_XYZ& XYZ) const;

public:

  mobiusCore_EXPORT core_Quaternion
    operator+(const core_Quaternion& Qn) const;

  mobiusCore_EXPORT core_Quaternion
    operator-(const core_Quaternion& Qn) const;

  mobiusCore_EXPORT core_Quaternion
    operator*(const core_Quaternion& Qn) const;

  mobiusCore_EXPORT core_Quaternion
    operator*(const core_XYZ& Qn_im) const;

  mobiusCore_EXPORT core_Quaternion
    operator*(const double k) const;

  mobiusCore_EXPORT double
    Dot(const core_Quaternion& Qn) const;

  mobiusCore_EXPORT core_Quaternion
    Cross(const core_Quaternion& Qn) const;

  mobiusCore_EXPORT core_Quaternion&
    operator=(const core_Quaternion& Qn);

  mobiusCore_EXPORT void
    Invert();

  mobiusCore_EXPORT core_Quaternion
    Inverted() const;

  mobiusCore_EXPORT void
    Conjugate();

  mobiusCore_EXPORT core_Quaternion
    Conjugated() const;

private:

  double q0, qx, qy, qz; //!< Quaternion components.

};

};

#endif
