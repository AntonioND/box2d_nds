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

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2StackAllocator.h>

b2ContactSolver::b2ContactSolver(const b2TimeStep& step, b2Contact** contacts, int32_t contactCount, b2StackAllocator* allocator)
{
	m_step = step;
	m_allocator = allocator;

	m_constraintCount = 0;
	for (int32_t i = 0; i < contactCount; ++i)
	{
		b2Assert(contacts[i]->IsSolid());
		m_constraintCount += contacts[i]->GetManifoldCount();
	}

	m_constraints = (b2ContactConstraint*)m_allocator->Allocate(m_constraintCount * sizeof(b2ContactConstraint));

	int32_t count = 0;
	for (int32_t i = 0; i < contactCount; ++i)
	{
		b2Contact* contact = contacts[i];

		b2Body* b1 = contact->m_shape1->m_body;
		b2Body* b2 = contact->m_shape2->m_body;
		int32_t manifoldCount = contact->GetManifoldCount();
		b2Manifold* manifolds = contact->GetManifolds();
		b2float32 friction = contact->m_friction;
		b2float32 restitution = contact->m_restitution;

		b2Vec2 v1 = b1->m_linearVelocity;
		b2Vec2 v2 = b2->m_linearVelocity;
		b2float32 w1 = b1->m_angularVelocity;
		b2float32 w2 = b2->m_angularVelocity;

		for (int32_t j = 0; j < manifoldCount; ++j)
		{
			b2Manifold* manifold = manifolds + j;

			b2Assert(manifold->pointCount > 0);

			const b2Vec2 normal = manifold->normal;

			b2Assert(count < m_constraintCount);
			b2ContactConstraint* c = m_constraints + count;
			c->body1 = b1;
			c->body2 = b2;
			c->manifold = manifold;
			c->normal = normal;
			c->pointCount = manifold->pointCount;
			c->friction = friction;
			c->restitution = restitution;

			for (int32_t k = 0; k < c->pointCount; ++k)
			{
				b2ManifoldPoint* cp = manifold->points + k;
				b2ContactConstraintPoint* ccp = c->points + k;

				ccp->normalForce = cp->normalForce;
				ccp->tangentForce = cp->tangentForce;
				ccp->separation = cp->separation;
				ccp->positionImpulse = 0.0f;

				ccp->localAnchor1 = cp->localPoint1;
				ccp->localAnchor2 = cp->localPoint2;

				b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
				b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());

				b2float32 r1Sqr = b2Dot(r1, r1);
				b2float32 r2Sqr = b2Dot(r2, r2);
				b2float32 rn1 = b2Dot(r1, normal);
				b2float32 rn2 = b2Dot(r2, normal);

				b2float32 kNormal = b1->m_invMass + b2->m_invMass;
				kNormal += b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_invI * (r2Sqr - rn2 * rn2);

#ifdef TARGET_FLOAT32_IS_FIXED
				b2Assert(b2Abs(kNormal) > 0);
				if(kNormal <= FLOAT32_EPSILON)
				{
					kNormal *= 64;
					ccp->normalMass = (1.0f / kNormal) * 64;
				}
				else
				{
					ccp->normalMass = 1.0f / kNormal;
				}
#else
				b2Assert(kNormal > FLOAT32_EPSILON);
				ccp->normalMass = 1.0f / kNormal;
#endif

				b2float32 kEqualized = b1->m_mass * b1->m_invMass + b2->m_mass * b2->m_invMass;
				kEqualized += b1->m_mass * b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_mass * b2->m_invI * (r2Sqr - rn2 * rn2);

#ifdef TARGET_FLOAT32_IS_FIXED
				b2Assert(b2Abs(kEqualized) > 0);
				if(kEqualized < FLOAT32_EPSILON)
				{
					kEqualized *= 64;
					ccp->equalizedMass = (1.0f / kEqualized) * 64;
				}
				else
				{
					ccp->equalizedMass = 1.0f / kEqualized;
				}
				
#else
				b2Assert(kEqualized > FLOAT32_EPSILON);
				ccp->equalizedMass = 1.0f / kEqualized;
#endif

				b2Vec2 tangent = b2Cross(normal, 1.0f);

				b2float32 rt1 = b2Dot(r1, tangent);
				b2float32 rt2 = b2Dot(r2, tangent);
				b2float32 kTangent = b1->m_invMass + b2->m_invMass;
				kTangent += b1->m_invI * (r1Sqr - rt1 * rt1) + b2->m_invI * (r2Sqr - rt2 * rt2);

#ifdef TARGET_FLOAT32_IS_FIXED
				b2Assert(b2Abs(kTangent) > 0);
				if(kTangent < FLOAT32_EPSILON)
				{
					kTangent *= 64;
					ccp->tangentMass = (1.0f /  kTangent) * 64;
				}
				else
				{
					ccp->tangentMass = 1.0f /  kTangent;
				}
#else
				b2Assert(kTangent > FLOAT32_EPSILON);
				ccp->tangentMass = 1.0f /  kTangent;
#endif

				// Setup a velocity bias for restitution.
				ccp->velocityBias = 0.0f;
				if (ccp->separation > 0.0f)
				{
					ccp->velocityBias = -60.0f * ccp->separation; // TODO_ERIN b2TimeStep
				}

				b2float32 vRel = b2Dot(c->normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
				if (vRel < -b2_velocityThreshold)
				{
					ccp->velocityBias += -c->restitution * vRel;
				}
			}

			++count;
		}
	}

	b2Assert(count == m_constraintCount);
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_constraints);
}

void b2ContactSolver::InitVelocityConstraints()
{
	// Warm start.
	for (int32_t i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;

		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		b2float32 invMass1 = b1->m_invMass;
		b2float32 invI1 = b1->m_invI;
		b2float32 invMass2 = b2->m_invMass;
		b2float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		if (b2World::s_enableWarmStarting)
		{
			for (int32_t j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				b2Vec2 P = m_step.dt * (ccp->normalForce * normal + ccp->tangentForce * tangent);
				b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
				b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());
				b1->m_angularVelocity -= invI1 * b2Cross(r1, P);
				b1->m_linearVelocity -= invMass1 * P;
				b2->m_angularVelocity += invI2 * b2Cross(r2, P);
				b2->m_linearVelocity += invMass2 * P;
			}
		}
		else
		{
			for (int32_t j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				ccp->normalForce = 0.0f;
				ccp->tangentForce = 0.0f;
			}
		}
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32_t i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		b2float32 invMass1 = b1->m_invMass;
		b2float32 invI1 = b1->m_invI;
		b2float32 invMass2 = b2->m_invMass;
		b2float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		// Solve normal constraints
		for (int32_t j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute normal force
			b2float32 vn = b2Dot(dv, normal);
			b2float32 lambda = - m_step.inv_dt * ccp->normalMass * (vn - ccp->velocityBias);

			// b2Clamp the accumulated force
			b2float32 newForce = b2Max(ccp->normalForce + lambda, 0.0f);
			lambda = newForce - ccp->normalForce;

			// Apply contact impulse
			b2Vec2 P = m_step.dt * lambda * normal;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->normalForce = newForce;
		}

		// Solve tangent constraints
		for (int32_t j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute tangent force
			b2float32 vt = b2Dot(dv, tangent);
			b2float32 lambda = m_step.inv_dt * ccp->tangentMass * (-vt);

			// b2Clamp the accumulated force
			b2float32 maxFriction = c->friction * ccp->normalForce;
			b2float32 newForce = b2Clamp(ccp->tangentForce + lambda, -maxFriction, maxFriction);
			lambda = newForce - ccp->tangentForce;

			// Apply contact impulse
			b2Vec2 P = m_step.dt * lambda * tangent;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->tangentForce = newForce;
		}
	}
}

void b2ContactSolver::FinalizeVelocityConstraints()
{
	for (int32_t i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Manifold* m = c->manifold;

		for (int32_t j = 0; j < c->pointCount; ++j)
		{
			m->points[j].normalForce = c->points[j].normalForce;
			m->points[j].tangentForce = c->points[j].tangentForce;
		}
	}
}

bool b2ContactSolver::SolvePositionConstraints(b2float32 baumgarte)
{
	b2float32 minSeparation = 0.0f;

	for (int32_t i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		b2float32 invMass1 = b1->m_mass * b1->m_invMass;
		b2float32 invI1 = b1->m_mass * b1->m_invI;
		b2float32 invMass2 = b2->m_mass * b2->m_invMass;
		b2float32 invI2 = b2->m_mass * b2->m_invI;
		
		b2Vec2 normal = c->normal;

		// Solver normal constraints
		for (int32_t j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());

			b2Vec2 p1 = b1->m_sweep.c + r1;
			b2Vec2 p2 = b2->m_sweep.c + r2;
			b2Vec2 dp = p2 - p1;

			// Approximate the current separation.
			b2float32 separation = b2Dot(dp, normal) + ccp->separation;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			b2float32 C = baumgarte * b2Clamp(separation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);

			// Compute normal impulse
			b2float32 dImpulse = -ccp->equalizedMass * C;

			// b2Clamp the accumulated impulse
			b2float32 impulse0 = ccp->positionImpulse;
			ccp->positionImpulse = b2Max(impulse0 + dImpulse, 0.0f);
			dImpulse = ccp->positionImpulse - impulse0;

			b2Vec2 impulse = dImpulse * normal;

			b1->m_sweep.c -= invMass1 * impulse;
			b1->m_sweep.a -= invI1 * b2Cross(r1, impulse);
			b1->SynchronizeTransform();

			b2->m_sweep.c += invMass2 * impulse;
			b2->m_sweep.a += invI2 * b2Cross(r2, impulse);
			b2->SynchronizeTransform();
		}
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5f * b2_linearSlop;
}
