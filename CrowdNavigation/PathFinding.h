#ifndef _PATHFINDING_H_
#define _PATHFINDING_H_

#include <windows.h>
#include <GL/glut.h>
#include <iostream>
#include <string.h>
#include <sstream>
#include <set>
#include <cmath>
#include <time.h>
#include <stdio.h>

using namespace std;

#pragma comment(lib, "glut32.lib")

const float WORLD_WIDTH = 200.0;
const float WORLD_HEIGHT = 160.0;
const float WINDOW_WIDTH = 800.0;
const float WINDOW_HEIGHT = 500.0;
const float GRID_SIZE = 4.0;
const float MOVE_STEP = 1.8;
const int MAX_CHAR = 128;
const int MOVE_OBJECT_NUM = 200;
const int INFI = 0xFFFFFFF;

#define CALC_MOVE_SPEED(_obj, _objIdx) \
	((_obj).s * MOVE_STEP / gMap[(_objIdx).x][(_objIdx).y].c)

typedef enum EDirection
{
	ED_U = 3,
	ED_D = -3,
	ED_L = -30,
	ED_R = 30,
	ED_UL = -27,
	ED_UR = 33,
	ED_DL = -33,
	ED_DR = 27,
	ED_NULL = 10,
} EDirection;
typedef enum EGridType
{
	EGT_NORMAL = 0,
	EGT_OPEN = 1,
	EGT_CLOSE = 2,
	EGT_OBSTOCLE = 4,
	EGT_DESTINATION = 5,
	EGT_USED = 6
} EGridType;

typedef struct SGridInfo
{
	int c;	// cost
	int pl; // the length to target
	int d;	// direction
	int t;	// type of the grid
	SGridInfo()
	{
		c = 0;
		pl = INFI;
		d = ED_NULL;
		t = EGT_NORMAL;
	}
} SGridInfo;
typedef struct SPoint
{
	GLfloat x;
	GLfloat y;
	SPoint(GLfloat ax = 0.0f, GLfloat ay = 0.0f)
	{
		x = ax;
		y = ay;
	}
} SPoint;
typedef struct SCoordinate
{
	int x;
	int y;
	SCoordinate(int ax = 0, int ay = 0)
	{
		x = ax;
		y = ay;
	}
	const SCoordinate &operator=(const SCoordinate &d)
	{
		x = d.x;
		y = d.y;
		return *this;
	}
} SCoordinate;
typedef struct SColorRGB
{
	GLfloat r;
	GLfloat g;
	GLfloat b;
	SColorRGB(GLfloat ar, GLfloat ag, GLfloat ab)
	{
		r = ar;
		g = ag;
		b = ab;
	}
} SColorRGB;
typedef struct SOpenGridInfo
{
	SCoordinate c;
	int pl;

	SOpenGridInfo(const SCoordinate &ac, int l = 0)
	{
		c = ac;
		pl = l;
	}
	bool operator<(const SOpenGridInfo &o) const
	{
		return pl < o.pl;
	}
} SOpenGridInfo;
typedef struct SMoveObject
{
	SPoint p;
	float s;
	SMoveObject(const SPoint ap = SPoint(), float as = 20.0f)
	{
		p = ap;
		s = as;
	}
} SMoveObject;

string Num2String(int i);
SPoint Index2World(const SCoordinate &idx);
SPoint Pixel2World(const SPoint &pixel);
SCoordinate World2Index(const SPoint &p);

void Initial();
void PathFindDisplay();
void ReshapeWin(int w, int h);
void MouseClick(int button, int state, int x, int y);
void MouseMove(int x, int y);
void TimeerFunc(int value);
void InitMoveObject();
/*********************** draw map **********************/
void DrawMap(int hGridNum, int vGridNum);
void DrawObstacle(int hGridNum, int vGridNum);
void DrawFlowField(int hGridNum, int vGridNum);
void DrawMoveObject(int hGridNum, int vGridNum);
void DrawDestination(const SPoint &dIdx);
/*********************** draw shape **********************/
void DrawString(string strn);
void DrawQuads(const SPoint &ldp, const SPoint &urp, const SColorRGB &c, int mode);
void DrawLineSurroundQuads(const SCoordinate &idx);
void DrawPoint(const SPoint &p, GLint size, const SColorRGB &c);
void DrawArraw(const SPoint &sp, const SPoint &ep);

/*********************** find path **********************/
void CalcFlowField(const SCoordinate &d, int hGridNum, int vGridNum);
void ChangeObjectPosition();
#endif