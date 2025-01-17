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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

#include <Box2D/Common/b2Settings.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2PairManager.h>
#include <climits>

#ifdef TARGET_FLOAT32_IS_FIXED
#define	B2BROADPHASE_MAX	(USHRT_MAX/2)
#else
#define	B2BROADPHASE_MAX	USHRT_MAX

#endif

const uint16_t b2_invalid = B2BROADPHASE_MAX;
const uint16_t b2_nullEdge = B2BROADPHASE_MAX;
struct b2BoundValues;

struct b2Bound
{
	bool IsLower() const { return (value & 1) == 0; }
	bool IsUpper() const { return (value & 1) == 1; }

	uint16_t value;
	uint16_t proxyId;
	uint16_t stabbingCount;
};

struct b2Proxy
{
	uint16_t GetNext() const { return lowerBounds[0]; }
	void SetNext(uint16_t next) { lowerBounds[0] = next; }
	bool IsValid() const { return overlapCount != b2_invalid; }

	uint16_t lowerBounds[2], upperBounds[2];
	uint16_t overlapCount;
	uint16_t timeStamp;
	void* userData;
};

class b2BroadPhase
{
public:
	b2BroadPhase(const b2AABB& worldAABB, b2PairCallback* callback);
	~b2BroadPhase();

	// Use this to see if your proxy is in range. If it is not in range,
	// it should be destroyed. Otherwise you may get O(m^2) pairs, where m
	// is the number of proxies that are out of range.
	bool InRange(const b2AABB& aabb) const;

	// Create and destroy proxies. These call Flush first.
	uint16_t CreateProxy(const b2AABB& aabb, void* userData);
	void DestroyProxy(int32_t proxyId);

	// Call MoveProxy as many times as you like, then when you are done
	// call Commit to finalized the proxy pairs (for your time step).
	void MoveProxy(int32_t proxyId, const b2AABB& aabb);
	void Commit();

	// Get a single proxy. Returns NULL if the id is invalid.
	b2Proxy* GetProxy(int32_t proxyId);

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	int32_t Query(const b2AABB& aabb, void** userData, int32_t maxCount);

	void Validate();
	void ValidatePairs();

private:
	void ComputeBounds(uint16_t* lowerValues, uint16_t* upperValues, const b2AABB& aabb);

	bool TestOverlap(b2Proxy* p1, b2Proxy* p2);
	bool TestOverlap(const b2BoundValues& b, b2Proxy* p);

	void Query(int32_t* lowerIndex, int32_t* upperIndex, uint16_t lowerValue, uint16_t upperValue,
				b2Bound* bounds, int32_t boundCount, int32_t axis);
	void IncrementOverlapCount(int32_t proxyId);
	void IncrementTimeStamp();

public:
	friend class b2PairManager;

	b2PairManager m_pairManager;

	b2Proxy m_proxyPool[b2_maxProxies];
	uint16_t m_freeProxy;

	b2Bound m_bounds[2][2*b2_maxProxies];

	uint16_t m_queryResults[b2_maxProxies];
	int32_t m_queryResultCount;

	b2AABB m_worldAABB;
	b2Vec2 m_quantizationFactor;
	int32_t m_proxyCount;
	uint16_t m_timeStamp;

	static bool s_validate;
};


inline bool b2BroadPhase::InRange(const b2AABB& aabb) const
{
	b2Vec2 d = b2Max(aabb.lowerBound - m_worldAABB.upperBound, m_worldAABB.lowerBound - aabb.upperBound);
	return b2Max(d.x, d.y) < 0.0f;
}

inline b2Proxy* b2BroadPhase::GetProxy(int32_t proxyId)
{
	if (proxyId == b2_nullProxy || m_proxyPool[proxyId].IsValid() == false)
	{
		return NULL;
	}

	return m_proxyPool + proxyId;
}

#endif
