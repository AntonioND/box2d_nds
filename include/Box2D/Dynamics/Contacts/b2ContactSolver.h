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

#ifndef CONTACT_SOLVER_H
#define CONTACT_SOLVER_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Dynamics/b2World.h>

class b2Contact;
class b2Body;
class b2Island;
class b2StackAllocator;

struct b2ContactConstraintPoint
{
	b2Vec2 localAnchor1;
	b2Vec2 localAnchor2;
	b2float32 normalForce;
	b2float32 tangentForce;
	b2float32 positionImpulse;
	b2float32 normalMass;
	b2float32 tangentMass;
	b2float32 equalizedMass;
	b2float32 separation;
	b2float32 velocityBias;
};

struct b2ContactConstraint
{
	b2ContactConstraintPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	b2Manifold* manifold;
	b2Body* body1;
	b2Body* body2;
	b2float32 friction;
	b2float32 restitution;
	int32_t pointCount;
};

class b2ContactSolver
{
public:
	b2ContactSolver(const b2TimeStep& step, b2Contact** contacts, int32_t contactCount, b2StackAllocator* allocator);
	~b2ContactSolver();

	void InitVelocityConstraints();
	void SolveVelocityConstraints();
	void FinalizeVelocityConstraints();

	bool SolvePositionConstraints(b2float32 baumgarte);

	b2TimeStep m_step;
	b2StackAllocator* m_allocator;
	b2ContactConstraint* m_constraints;
	int m_constraintCount;
};

#endif
