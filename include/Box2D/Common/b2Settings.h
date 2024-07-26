/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_SETTINGS_H
#define B2_SETTINGS_H

#include <cassert>
#include <cstdint>

#define B2_NOT_USED(x) (void)x
#define b2Assert(A) assert(A)

#ifdef	TARGET_FLOAT32_IS_FIXED

#include <Box2D/Common/Fixed.h>

typedef Fixed b2float32;
#define	FLOAT32_MAX	FIXED_MAX
#define	FLOAT32_MIN	FIXED_MIN
#define	FLOAT32_EPSILON	FIXED_EPSILON

#else

typedef float b2float32;
#define	FLOAT32_MAX	FLT_MAX
#define	FLOAT32_MIN	FLT_MIN
#define	FLOAT32_EPSILON	FLT_EPSILON

#endif

const b2float32 b2_pi = 3.14159265359f;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision
const int32_t b2_maxManifoldPoints = 2;
const int32_t b2_maxPolygonVertices = 8;
const int32_t b2_maxProxies = 512;				// this must be a power of two
const int32_t b2_maxPairs = 8 * b2_maxProxies;	// this must be a power of two

// Dynamics

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
const b2float32 b2_linearSlop = 0.005f;	// 0.5 cm

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
const b2float32 b2_angularSlop = 2.0f / 180.0f * b2_pi;			// 2 degrees

/// Continuous collision detection (CCD) works with core, shrunken shapes. This is the
/// amount by which shapes are automatically shrunk to work with CCD. This must be
/// larger than b2_linearSlop.
const b2float32 b2_toiSlop = 8.0f * b2_linearSlop;

/// Maximum number of contacts to be handled to solve a TOI island.
const int32_t b2_maxTOIContactsPerIsland = 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
const b2float32 b2_velocityThreshold = 1.0f;		// 1 m/s

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
const b2float32 b2_maxLinearCorrection = 0.2f;	// 20 cm

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
const b2float32 b2_maxAngularCorrection = 8.0f / 180.0f * b2_pi;			// 8 degrees

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
#ifdef TARGET_FLOAT32_IS_FIXED
const b2float32 b2_maxLinearVelocity = 100.0f;
#else
const b2float32 b2_maxLinearVelocity = 200.0f;
#endif
const b2float32 b2_maxLinearVelocitySquared = b2_maxLinearVelocity * b2_maxLinearVelocity;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
const b2float32 b2_maxAngularVelocity = 250.0f;
const b2float32 b2_maxAngularVelocitySquared = b2_maxAngularVelocity * b2_maxAngularVelocity;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
const b2float32 b2_contactBaumgarte = 0.2f;

// Sleep

/// The time that a body must be still before it will go to sleep.
const b2float32 b2_timeToSleep = 0.5f;									// half a second

/// A body cannot sleep if its linear velocity is above this tolerance.
const b2float32 b2_linearSleepTolerance = 0.01f;		// 1 cm/s

/// A body cannot sleep if its angular velocity is above this tolerance.
const b2float32 b2_angularSleepTolerance = 2.0f / 180.0f;		// 2 degrees/s

// Memory Allocation

/// The current number of bytes allocated through b2Alloc.
extern int32_t b2_byteCount;

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32_t size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
	int32_t major;		///< significant changes
	int32_t minor;		///< incremental changes
	int32_t revision;		///< bug fixes
};

/// Current version.
extern b2Version b2_version;


#endif
