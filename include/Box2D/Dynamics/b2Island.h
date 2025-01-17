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

#ifndef B2_ISLAND_H
#define B2_ISLAND_H

#include <Box2D/Common/b2Math.h>

class b2Contact;
class b2Body;
class b2Joint;
class b2StackAllocator;
class b2ContactListener;
struct b2ContactConstraint;
struct b2TimeStep;

class b2Island
{
public:
	b2Island(int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity,
			b2StackAllocator* allocator, b2ContactListener* listener);
	~b2Island();

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Solve(const b2TimeStep& step, const b2Vec2& gravity, bool correctPositions, bool allowSleep);

	void SolveTOI(const b2TimeStep& subStep);

	void Add(b2Body* body)
	{
		b2Assert(m_bodyCount < m_bodyCapacity);
		m_bodies[m_bodyCount++] = body;
	}

	void Add(b2Contact* contact)
	{
		b2Assert(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact;
	}

	void Add(b2Joint* joint)
	{
		b2Assert(m_jointCount < m_jointCapacity);
		m_joints[m_jointCount++] = joint;
	}

	void Report(b2ContactConstraint* constraints);

	b2StackAllocator* m_allocator;
	b2ContactListener* m_listener;

	b2Body** m_bodies;
	b2Contact** m_contacts;
	b2Joint** m_joints;

	int32_t m_bodyCount;
	int32_t m_jointCount;
	int32_t m_contactCount;

	int32_t m_bodyCapacity;
	int32_t m_contactCapacity;
	int32_t m_jointCapacity;

	int32_t m_positionIterationCount;
};

#endif
