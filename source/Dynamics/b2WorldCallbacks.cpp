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

#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

b2ContactFilter b2_defaultFilter;

// Return true if contact calculations should be performed between these two shapes.
// If you implement your own collision filter you may want to build from this implementation.
bool b2ContactFilter::ShouldCollide(b2Shape* shape1, b2Shape* shape2)
{
	if (shape1->m_groupIndex == shape2->m_groupIndex && shape1->m_groupIndex != 0)
	{
		return shape1->m_groupIndex > 0;
	}

	bool collide = (shape1->m_maskBits & shape2->m_categoryBits) != 0 && (shape1->m_categoryBits & shape2->m_maskBits) != 0;
	return collide;
}

b2DebugDraw::b2DebugDraw()
{
	m_drawFlags = 0;
}

void b2DebugDraw::SetFlags(uint32_t flags)
{
	m_drawFlags = flags;
}

uint32_t b2DebugDraw::GetFlags() const
{
	return m_drawFlags;
}

void b2DebugDraw::AppendFlags(uint32_t flags)
{
	m_drawFlags |= flags;
}

void b2DebugDraw::ClearFlags(uint32_t flags)
{
	m_drawFlags &= ~flags;
}
