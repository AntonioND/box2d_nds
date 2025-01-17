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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			b2PolygonDef sd;
			sd.vertices[0].Set(-9.0f, -1.1f);
			sd.vertices[1].Set(7.0f, -1.1f);
			sd.vertices[2].Set(5.0f, -0.9f);
			sd.vertices[3].Set(-11.0f, -0.9f);
			sd.vertexCount = 4;
			sd.density = 0.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			m_body1 = m_world->CreateStaticBody(&bd);
			m_body1->CreateShape(&sd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(0.5f, 0.5f);
			sd.density = 1.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			m_body2 = m_world->CreateDynamicBody(&bd);
			m_body2->CreateShape(&sd);
			m_body2->SetMassFromShapes();
		}

		m_world->m_gravity.Set(0.0f, 0.0f);
		b2World::s_enablePositionCorrection = false;
	}

	~PolyCollision()
	{
		b2World::s_enablePositionCorrection = true;
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(Settings* settings)
	{
		settings->pause = 1;
		Test::Step(settings);
		settings->pause = 0;

		const b2XForm& xf1 = m_body1->GetXForm();
		for (b2Contact* c = m_world->m_contactList; c; c = c->m_next)
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				glPointSize(4.0f);
				glColor3f(1.0f, 0.0f, 0.0f);
				glBegin(GL_POINTS);
				for (int j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v = b2Mul(xf1, m->points[j].localPoint1);
					glVertex2f(v.x, v.y);
				}
				glEnd();
				glPointSize(1.0f);

				glColor3f(1.0f, 1.0f, 0.0f);
				glBegin(GL_LINES);
				for (int32 j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v1 = b2Mul(xf1, m->points[j].localPoint1);
					b2Vec2 v2 = v1 + 1.0f * m->normal;
					glVertex2f(v1.x, v1.y);
					glVertex2f(v2.x, v2.y);
				}
				glEnd();
			}
		}
	}

	void Keyboard(unsigned char key)
	{
		b2Vec2 p = m_body2->GetPosition();
		float32 a = m_body2->GetAngle();

		switch (key)
		{
		case 'a':
			p.x -= 0.1f;
			break;

		case 'd':
			p.x += 0.1f;
			break;

		case 's':
			p.y -= 0.1f;
			break;

		case 'w':
			p.y += 0.1f;
			break;

		case 'q':
			a += 0.1f * b2_pi;
			break;

		case 'e':
			a -= 0.1f * b2_pi;
			break;
		}

		m_body2->SetXForm(p, a);
	}

	b2Body* m_body1;
	b2Body* m_body2;
};

#endif
