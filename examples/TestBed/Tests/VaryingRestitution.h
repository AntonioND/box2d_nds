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

#ifndef VARYING_RESTITUTION_H
#define VARYING_RESTITUTION_H

class VaryingRestitution : public Test
{
public:

	VaryingRestitution()
	{
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);

			b2Body* ground = m_world->CreateStaticBody(&bd);
			ground->CreateShape(&sd);
		}

		{
			b2CircleDef sd;
			sd.radius = 1.0f;
			sd.density = 1.0f;

			float32 restitution[7] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

			for (int32 i = 0; i < 7; ++i)
			{
				b2BodyDef bd;
				bd.position.Set(-10.0f + 3.0f * i, 20.0f);

				b2Body* body = m_world->CreateDynamicBody(&bd);

				sd.restitution = restitution[i];
				body->CreateShape(&sd);
				body->SetMassFromShapes();
			}
		}
	}

	static Test* Create()
	{
		return new VaryingRestitution;
	}
};

#endif
