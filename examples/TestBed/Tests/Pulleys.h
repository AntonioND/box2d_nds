/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#ifndef PULLEYS_H
#define PULLEYS_H

class Pulleys : public Test
{
public:
	Pulleys()
	{
		b2Body* ground = NULL;
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			ground = m_world->CreateStaticBody(&bd);
			ground->CreateShape(&sd);
		}

		{
			float32 a = 2.0f;
			float32 b = 4.0f;
			float32 y = 16.0f;
			float32 L = 12.0f;

			b2PolygonDef sd;
			sd.SetAsBox(a, b);
			sd.density = 5.0f;

			b2BodyDef bd;

			bd.position.Set(-10.0f, y);
			b2Body* body1 = m_world->CreateDynamicBody(&bd);
			body1->CreateShape(&sd);
			body1->SetMassFromShapes();

			bd.position.Set(10.0f, y);
			b2Body* body2 = m_world->CreateDynamicBody(&bd);
			body2->CreateShape(&sd);
			body2->SetMassFromShapes();

			b2PulleyJointDef pulleyDef;
			b2Vec2 anchor1(-10.0f, y + b);
			b2Vec2 anchor2(10.0f, y + b);
			b2Vec2 groundAnchor1(-10.0f, y + b + L);
			b2Vec2 groundAnchor2(10.0f, y + b + L);
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0f);

			m_joint1 = (b2PulleyJoint*)m_world->CreateJoint(&pulleyDef);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 0:
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		float32 ratio = m_joint1->GetRatio();
		float32 L = m_joint1->GetLength1() + ratio * m_joint1->GetLength2();
		DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", (float) ratio, (float) L);
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new Pulleys;
	}

	b2PulleyJoint* m_joint1;
};

#endif
