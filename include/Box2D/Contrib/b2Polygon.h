/*
 * Copyright (c) 2007 Eric Jordan
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

#ifndef B2_POLYGON_H
#define B2_POLYGON_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Contrib/b2Triangle.h>
#include <Box2D/Dynamics/b2World.h>
#include <cstring>

class b2Polygon;

int32_t remainder(int32_t x, int32_t modulus);
int32_t TriangulatePolygon(float32* xv, float32* yv, int32_t vNum, b2Triangle* results);
bool IsEar(int32_t i, float32* xv, float32* yv, int32_t xvLength); //Not for external use
int32_t PolygonizeTriangles(b2Triangle* triangulated, int32_t triangulatedLength, b2Polygon* polys, int32_t polysLength);
int32_t DecomposeConvex(b2Polygon* p, b2Polygon* results, int32_t maxPolys);
b2PolygonDef* DecomposeConvexAndAddTo(b2World *world, b2Polygon* p, b2Body* body, b2PolygonDef* prototype);
b2Polygon ConvexHull(b2Vec2* v, int nVert);
b2Polygon ConvexHull(float32* cloudX, float32* cloudY, int32_t nVert);
void ReversePolygon(float32* x, float32* y, int n);

b2Polygon *TraceEdge(b2Polygon* p); //For use with self-intersecting polygons, finds outline

class b2Polygon {
	
public:
    const static int32_t maxVerticesPerPolygon = b2_maxPolygonVertices;

    float32* x; //vertex arrays
    float32* y;
    int32_t nVertices;
	
	float32 area;
	bool areaIsSet;
	
    b2Polygon(float32* _x, float32* _y, int32_t nVert);
    b2Polygon(b2Vec2* v, int32_t nVert);
	b2Polygon();
    ~b2Polygon();
	
	float32 GetArea();
	
	void MergeParallelEdges(float32 tolerance);
    b2Vec2* GetVertexVecs();
    b2Polygon(b2Triangle& t);
    void Set(const b2Polygon& p);
    bool IsConvex();
	bool IsCCW();
	bool IsUsable(bool printError);
	bool IsUsable();
    bool IsSimple();
    void AddTo(b2PolygonDef& pd);
	
    b2Polygon* Add(b2Triangle& t);

	void print(){
		printFormatted();
//		for (int32_t i=0; i<nVertices; ++i){
//			printf("i: %d, x:%f, y:%f\n",i,x[i],y[i]);
//		}
	}

	void printFormatted(){
		/*
		printf("float32 xv[] = {");
		for (int32_t i=0; i<nVertices; ++i){
			printf("%ff,",x[i]);
		}
		printf("};\nfloat32 yv[] = {");
		for (int32_t i=0; i<nVertices; ++i){
			printf("%ff,",y[i]);
		}
		printf("};\n");
		*/
	}
    
	b2Polygon(const b2Polygon& p){
		nVertices = p.nVertices;
		area = p.area;
		areaIsSet = p.areaIsSet;
		x = new float32[nVertices];
		y = new float32[nVertices];
		memcpy(x, p.x, nVertices * sizeof(float32));
		memcpy(y, p.y, nVertices * sizeof(float32));
	}

	
};

const int32_t MAX_CONNECTED = 32;
class b2PolyNode{
public:
	b2Vec2 position;
	b2PolyNode* connected[MAX_CONNECTED];
	int32_t nConnected;

	b2PolyNode(b2Vec2& pos);
	b2PolyNode();
	void AddConnection(b2PolyNode& toMe);
	void RemoveConnection(b2PolyNode& fromMe);
	void RemoveConnectionByIndex(int32_t index);
	bool IsConnectedTo(b2PolyNode& me);
	b2PolyNode* GetRightestConnection(b2PolyNode& incoming);
	b2PolyNode* GetRightestConnection(b2Vec2& incomingDir);
};

#endif
