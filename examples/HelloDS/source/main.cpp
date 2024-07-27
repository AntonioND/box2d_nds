/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
* Copyright (c) 2024 Antonio Niño Díaz
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

#include <Box2D/Box2D.h>

#include <cstdio>

#include <gl2d.h>
#include <nds.h>

const uint32_t screen_width = 256;
const uint32_t screen_height = 192;

typedef struct {
    float h, w;

    b2BodyDef bodyDef;
    b2Body* body;
    b2PolygonDef shapeDef;
} WallInfo;

WallInfo NewWall(float x, float y, float w, float h, float angle, b2World &world)
{
    WallInfo Wall = { 0 };

    // Define the ground body.
    Wall.bodyDef.position.Set(x, y);
    Wall.bodyDef.angle = angle;

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    Wall.body = world.CreateStaticBody(&(Wall.bodyDef));

    // Define the ground box shape.

    // The extents are the half-widths of the box.
    Wall.shapeDef.SetAsBox(w, h);

    // Add the ground shape to the ground body.
    Wall.body->CreateShape(&(Wall.shapeDef));

    Wall.w = w;
    Wall.h = h;

    return Wall;
}

typedef struct {
    float h, w;

    b2BodyDef bodyDef;
    b2Body* body;
    b2PolygonDef shapeDef;
} BoxInfo;

BoxInfo NewBox(float x, float y, float w, float h, float angleSpeed, b2World &world)
{
    BoxInfo Box = { 0 };

    // Define the dynamic body. We set its position and call the body factory.
    Box.bodyDef.position.Set(x, y);
    Box.body = world.CreateDynamicBody(&(Box.bodyDef));

    // Define another box shape for our dynamic body.
    Box.shapeDef.SetAsBox(w, h);

    // Set the box density to be non-zero, so it will be dynamic.
    Box.shapeDef.density = 1.0f;

    // Override the default friction.
    Box.shapeDef.friction = 0.3f;

    // Add the shape to the body.
    Box.body->CreateShape(&(Box.shapeDef));

    // Now tell the dynamic body to compute it's mass properties base
    // on its shape.
    Box.body->SetMassFromShapes();

    Box.body->SetAngularVelocity(angleSpeed);
    // Box.body->m_angularDamping = 0.01f;

    Box.w = w;
    Box.h = h;

    return Box;
}

static int fps = 0;
static int frame_count = 0;

void timer0_handler(void)
{
    fps = frame_count;
    frame_count = 0;
}

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
int main(int argc, char** argv)
{
    consoleDemoInit();

    videoSetMode(MODE_0_3D);

    // Call this function every second
    timerStart(0, ClockDivider_1024, timerFreqToTicks_1024(1), timer0_handler);

    // Initialize OpenGL to some sensible defaults
    glScreen2D();

    // Define the size of the world. Simulation will still work
    // if bodies reach the end of the world, but it will be slower.
    b2AABB worldAABB;
    worldAABB.lowerBound.Set(-100.0f, -100.0f);
    worldAABB.upperBound.Set(100.0f, 100.0f);

    // Define the gravity vector.
    b2Vec2 gravity(0.0f, -10.0f);

    // Do we want to let bodies sleep?
    bool doSleep = true;

    // Construct a world object, which will hold and simulate the rigid bodies.
    b2World world(worldAABB, gravity, doSleep);

    int numWalls = 3;
    WallInfo Wall[numWalls];
    Wall[0] = NewWall(40.0, -2.0, 40.0, 4.0, 0, world);
    Wall[1] = NewWall(80.0, 20.0, 4.0, 30.0, 0, world);
    Wall[2] = NewWall(29.0, 17.0, 12.0, 3.0, M_PI * 1 / 4, world);
    //Wall[3] = NewWall(-2.0, 20.0, 4.0, 30.0, 0, world); // Left wall

    int numBoxesX = 4;
    int numBoxesY = 4;
    int numBoxes = numBoxesX * numBoxesY;
    BoxInfo Box[numBoxes];
    for (int j = 0; j < numBoxesY; j++)
    {
        for (int i = 0; i < numBoxesX; i++)
        {
            int index = i + j * numBoxesX;
            float x = i * 9.0 + 15.0 + j * 2.0;
            float y = j * 9.0 + 35.0;
            Box[index] = NewBox(x, y, i / 2 + 1, 4 - j / 2, index - 8, world);
        }
    }

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    b2float32 timeStep = 1.0f / 60.0f;
    int32_t iterations = 6;

    // This is our little game loop.
    while (1)
    {
        // Instruct the world to perform a single step of simulation. It is
        // generally best to keep the time step and iterations fixed.
        world.Step(timeStep, iterations);

        // Set up GL2D for 2D mode
        glBegin2D();

        float scale = 3;

        for (int i = 0; i < numWalls; i++)
        {
            glPushMatrix();
            {
                // Now print the position and angle of the body.
                b2Vec2 position = Wall[i].body->GetPosition();
                b2float32 angle = Wall[i].body->GetAngle();

                int w = Wall[i].w * scale;
                int h = Wall[i].h * scale;
                int xc = scale * float(position.x);
                int yc = scale * float(position.y);

                glTranslate3f32(xc, screen_height - yc, 0);

                glRotateZ(float(angle) * -(360.0 / (2.0 * M_PI)));

                glBoxFilled(-w, -h, w, h, RGB15(0, 0, (i + 1) * 10));
                glBoxFilled(-1, -1, 1, 1, RGB15(31, 31, 31));
            }
            glPopMatrix(1);
        }

        for (int i = 0; i < numBoxes; i++)
        {
            glPushMatrix();
            {
                // Now print the position and angle of the body.
                b2Vec2 position = Box[i].body->GetPosition();
                b2float32 angle = Box[i].body->GetAngle();

                int w = Box[i].w * scale;
                int h = Box[i].h * scale;
                int xc = scale * float(position.x);
                int yc = scale * float(position.y);

                glTranslate3f32(xc, screen_height - yc, 0);

                glRotateZ(-float(angle) * -(360.0 / (2.0 * M_PI)));

                glBoxFilled(-w, -h, w, h, RGB15((i + 1) * 10, 0, 0));
                glBoxFilled(-1, -1, 1, 1, RGB15(31, 31, 31));

            }
            glPopMatrix(1);
        }

        glEnd2D();

        glFlush(0);

        frame_count++;

        swiWaitForVBlank();

        printf("\x1b[0;0HFPS: %d \n", fps);
    }

    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    return 0;
}
