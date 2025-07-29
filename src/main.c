#include "raylib.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "rlgl.h"
#include "raymath.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h" 

#include "resource_dir.h"	// utility header for SearchAndSetResourceDir
#include <math.h>

// static inline float maxd(const int a, const int b) {
//     return (a > b ? a : b);
// }
#define max(a, b) ((a) > (b) ? (a) : (b))

typedef struct Pose {
	float x;
	float y;
	float theta;
} Pose;


typedef struct GridVector2 {
	int x;
	int y;
} GridVector2;


#define NUM_OBSTACLES 10

#define GRID_WIDTH 10
#define GRID_HEIGHT 20
#define SCREEN_SIZE 800
#define SPACING 10

// If you want to manually specify the grid spacing for vizualization uncomment this line
// #define GRID_SIZE 20

#define HAS_LIDAR

#ifndef GRID_SIZE
// Will find the grid size necessary to have everything plotted within the specific screen size, with padding
#define GRID_SIZE (SCREEN_SIZE - 2 * SPACING) / max(GRID_HEIGHT, GRID_WIDTH)

#endif

#ifdef HAS_LIDAR
#define NUM_LIDAR_SCANS 100 // Numer of datapoints inside the LiDAR scan
#define LIDAR_FOV       (2*PI) // The view of the lidar in radians
#define LIDAR_RANGE     1.0f // Define lidar range in meters 

#endif

#define REAL_GRID_SIZE 0.3f // Represents the true size of a grid cell in meters
#define REAL_GRID_SIZE_2 REAL_GRID_SIZE / 2.0f // Represents the true size of a grid cell in meters

#define PIXEL_SCALE  GRID_SIZE / REAL_GRID_SIZE // Represents the gridsize to pixel scaling for the motion controlling in meters

#define REAL_PLAYER_SIZE 0.2f // Dimension of the robot in meters
#define REAL_PLAYER_SIZE_2 REAL_PLAYER_SIZE / 2.0f
#define PLAYER_SIZE REAL_PLAYER_SIZE * PIXEL_SCALE // Size of the robot in pixel size

const GridVector2 SnapToGridVec(const Vector2* pose){
	return (GridVector2) {(int) floorf(pose->x / REAL_GRID_SIZE), (int) floorf(pose->y / REAL_GRID_SIZE)};
}

const GridVector2 SnapToGridPose(const Pose* pose){
	return (GridVector2) {(int) floorf(pose->x / REAL_GRID_SIZE), (int)floorf(pose->y / REAL_GRID_SIZE)};
}

GridVector2 GridVector2Add(const GridVector2 v1,const GridVector2 v2)
{
    return (GridVector2) { v1.x + v2.x, v1.y + v2.y };
}

Vector2 Vector2MultiplyScalar(const Vector2 v1,const float v2)
{
    return (Vector2)  { (float)v1.x*v2, (float) v1.y*v2 };
}


void GenerateObstacles(int occupancyGrid[][GRID_HEIGHT], int obtacleLocations[][2], Rectangle * obstacleCollisions, Vector2 obstacleVertices[][4]) {
	for(int i= 0; i < NUM_OBSTACLES; ++i){
		int rx, ry;

		do{
			rx = rand() % GRID_WIDTH; 
			ry = rand() % GRID_HEIGHT; 
		}while(occupancyGrid[rx][ry] != -1);

		occupancyGrid[rx][ry] = i;

		obtacleLocations[i][0] = rx;      // Returns a pseudo-random integer between 0 and gridWidth.
		obtacleLocations[i][1] = ry; // Returns a pseudo-random integer between 0 and gridHeight.

		obstacleCollisions[i].x = (float)rx * GRID_SIZE;
		obstacleCollisions[i].y = (float)ry * GRID_SIZE;
		obstacleCollisions[i].width = (float) GRID_SIZE;
		obstacleCollisions[i].height = (float) GRID_SIZE;

		const float realRx = rx * REAL_GRID_SIZE;
		const float realRy = ry * REAL_GRID_SIZE;

		obstacleVertices[i][0] = (Vector2){realRx                 , realRy};
		obstacleVertices[i][1] = (Vector2){realRx + REAL_GRID_SIZE, realRy};
		obstacleVertices[i][2] = (Vector2){realRx + REAL_GRID_SIZE, realRy + REAL_GRID_SIZE};
		obstacleVertices[i][3] = (Vector2){realRx                 , realRy + REAL_GRID_SIZE};

		printf("Obstacle %d: (%d, %d)\n", i, rx, ry);		
	}
}

void CameraLogic(Camera2D* camera, int* zoomMode){
// Update
        //----------------------------------------------------------------------------------
        if (IsKeyPressed(KEY_ONE)) *zoomMode = 0;
        else if (IsKeyPressed(KEY_TWO)) *zoomMode = 1;

		 // Translate based on mouse right click
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 delta = GetMouseDelta();
            delta = Vector2Scale(delta, -1.0f/ camera->zoom);
            camera->target = Vector2Add(camera->target, delta);
        }

		 if (*zoomMode == 0)
        {
            // Zoom based on mouse wheel
            float wheel = GetMouseWheelMove();
            if (wheel != 0)
            {
                // Get the world point that is under the mouse
                Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), *camera);

                // Set the offset to where the mouse is
                camera->offset = GetMousePosition();

                // Set the target to match, so that the camera maps the world space point 
                // under the cursor to the screen space point under the cursor at any zoom
                camera->target = mouseWorldPos;

                // Zoom increment
                // Uses log scaling to provide consistent zoom speed
                float scale = 0.2f*wheel;
                camera->zoom = Clamp(expf(logf(camera->zoom)+scale), 0.125f, 64.0f);
            }
        }
        else
        {
            // Zoom based on mouse right click
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
            {
                // Get the world point that is under the mouse
                Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), *camera);

                // Set the offset to where the mouse is
                camera->offset = GetMousePosition();

                // Set the target to match, so that the camera maps the world space point 
                // under the cursor to the screen space point under the cursor at any zoom
                camera->target = mouseWorldPos;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
            {
                // Zoom increment
                // Uses log scaling to provide consistent zoom speed
                float deltaX = GetMouseDelta().x;
                float scale = 0.005f*deltaX;
                camera->zoom = Clamp(expf(logf(camera->zoom)+scale), 0.125f, 64.0f);
            }
        }
}


#ifdef HAS_LIDAR
// static float   lidarDistances[NUM_LIDAR_SCANS];
// static Vector2 lidarHits[NUM_LIDAR_SCANS];

// Ray-edge intersection: returns t along ray (dir normalized) or FLT_MAX if none
static inline float RayEdgeIntersect(
    const Vector2 orig, const Vector2 dir,
    const Vector2 a, const Vector2 b)
{
    // Compute intersection of ray (orig + t*dir) with segment a->b
    float ex = b.x - a.x;
    float ey = b.y - a.y;
    // denom = cross(edge, -dir)
    float denom = ex * -dir.y - ey * -dir.x;
    if (fabsf(denom) < 1e-6f) return FLT_MAX;
    float dx = a.x - orig.x;
    float dy = a.y - orig.y;
    float s = (dx * -dir.y - dy * -dir.x) / denom;
    if (s < 0.0f || s > 1.0f) return FLT_MAX;
    float t = (dx * ey - dy * ex) / denom;
    return (t >= 0.0f) ? t : FLT_MAX;
}

#define LIDAR_STEP LIDAR_FOV / ((float) NUM_LIDAR_SCANS - 1)

// DDA grid traversal: cast ray in grid, test encountered obstacle cells
void PerformRayCasting(Vector2 origin, float startAng, float* distances, Vector2* hits, const int occupancyGrid[][GRID_HEIGHT], const Vector2 obstacleVerts[][4]) {
    for(int i=0;i<NUM_LIDAR_SCANS;i++) {
        float ang = startAng + i*LIDAR_STEP;
        Vector2 dir = { cosf(ang), sinf(ang) };
        // initialize ray position in grid coords

        int gx = (int)(origin.x * (1.0f / REAL_GRID_SIZE));
        int gy = (int)(origin.y * (1.0f / REAL_GRID_SIZE));
        
		// DDA setup

        int stepX = (dir.x >= 0.0f) ? 1 : -1;
        int stepY = (dir.y >= 0.0f) ? 1 : -1;

         float inv_dx = (dir.x != 0.0f) ? (1.0f / dir.x) : 0.0f;
        float inv_dy = (dir.y != 0.0f) ? (1.0f / dir.y) : 0.0f;

        float tDeltaX = (dir.x != 0.0f) ? fabsf(REAL_GRID_SIZE * inv_dx) : FLT_MAX;
        float tDeltaY = (dir.y != 0.0f) ? fabsf(REAL_GRID_SIZE * inv_dy) : FLT_MAX;

        float cellOriginX = gx * REAL_GRID_SIZE;
        float cellOriginY = gy * REAL_GRID_SIZE;
        float rx = origin.x - cellOriginX;
        float ry = origin.y - cellOriginY;

        float tMaxX = (dir.x > 0.0f)
            ? ((REAL_GRID_SIZE - rx) * inv_dx)
            : ((dir.x < 0.0f) ? (-rx * inv_dx) : FLT_MAX);
        float tMaxY = (dir.y > 0.0f)
            ? ((REAL_GRID_SIZE - ry) * inv_dy)
            : ((dir.y < 0.0f) ? (-ry * inv_dy) : FLT_MAX);

        // initialize best hit at max range
        float maxR = LIDAR_RANGE;
        float bestT = maxR;
        Vector2 bestHit = { origin.x + dir.x * maxR,
                            origin.y + dir.y * maxR };

        float traveled = 0.0f;
        // traverse grid
        while (traveled < bestT) {
            // cell check
            if ((unsigned)gx < GRID_WIDTH && (unsigned)gy < GRID_HEIGHT) {
                int cellId = occupancyGrid[gx][gy];
                if (cellId >= 0) {
                    // obstacle edges
                    const Vector2 *poly = obstacleVerts[cellId];
                    for (int e = 0; e < 4; ++e) {
                        float t = RayEdgeIntersect(
                            origin, dir,
                            poly[e], poly[(e + 1) & 3]
                        );
                        if (t < bestT) {
                            bestT = t;
                            bestHit.x = origin.x + dir.x * t;
                            bestHit.y = origin.y + dir.y * t;
                        }
                    }
                }

                // advance
                if (tMaxX < tMaxY) {
                    gx += stepX;
                    traveled = tMaxX;
                    tMaxX += tDeltaX;
                } else {
                    gy += stepY;
                    traveled = tMaxY;
                    tMaxY += tDeltaY;
                }
            } else break;
        }
        // finalize
        distances[i] = (bestT < maxR) ? bestT : -1.0f;
        hits[i] = bestHit;
    }
}

#if NUM_OBSTACLES > 0
void UpdateLidarScan(const Pose *robotPosition, float* lidarScan, Vector2* lidarHits, const Vector2* worldBorder, const int occupancyGrid[][GRID_HEIGHT],const Vector2 obstacleVerts[][4]){
	//TODO Have Ray Casting Performed such that it gets the closest hit point to an obstacle and populates the lidarScan array
    Vector2 origin = { robotPosition->x, robotPosition->y };
    float startAng = robotPosition->theta - 0.5f * LIDAR_FOV;

	PerformRayCasting(origin, startAng, lidarScan, lidarHits, occupancyGrid, obstacleVerts);
}
#else 
void UpdateLidarScan(const Pose *robotPosition, float* lidarScan, Vector2* lidarHits, const Vector2* worldBorder){
	//TODO Have Ray Casting Performed such that it gets the closest hit point to an obstacle and populates the lidarScan array

}
#endif


void DrawLiDAR(const Pose *robotPosition, float lidarScan[], Vector2 lidarHits[]){
	Vector2 player = (Vector2){robotPosition->x * PIXEL_SCALE, robotPosition->y* PIXEL_SCALE};

	for(int i =0; i < NUM_LIDAR_SCANS; ++i){
		DrawLineEx(player, Vector2MultiplyScalar(lidarHits[i], PIXEL_SCALE), 2.0f ,GREEN);
	}
}
#endif

#define TEXT_OFFSET 1
#define TEXT_FONT_SIZE 10

void DrawWorldGrid(){
	// Draw centered world grid

	for (int i = 0; i < GRID_WIDTH + 1; ++i)
	{
		DrawLineV((Vector2){(float)(GRID_SIZE *i ), 0}, (Vector2){ (float)(GRID_SIZE*i), (float) (GRID_SIZE * (GRID_HEIGHT ) )}, LIGHTGRAY);
	}

	for (int i = 0; i < GRID_HEIGHT + 1; ++i)
	{
		DrawLineV((Vector2){0, (float)(GRID_SIZE*i )}, (Vector2){ (float)(GRID_SIZE * (GRID_WIDTH )  ), (float)(GRID_SIZE *i)}, LIGHTGRAY);
	}
}

void DrawWorldGridText(){
	// Since we are flipping the world y axis we want to ensure that we do not flip the text itself

	for (int r = 0; r < GRID_HEIGHT; r++)
	{
		for (int c = 0; c < GRID_WIDTH; ++c)
		{
			// TODO figure out how to have font size be dynamically set based on the GRID_SIZE variable to ensure that it all fits inside it
			// DrawText(TextFormat("[%i,%i]", c, r), GRID_SIZE*c + 2,(GRID_HEIGHT-1)* GRID_SIZE -  GRID_SIZE*r + 2, TEXT_FONT_SIZE, LIGHTGRAY);
			DrawText(TextFormat("[%i,%i]", c, r), GRID_SIZE*c + 2,(GRID_HEIGHT)* GRID_SIZE -  GRID_SIZE*r + 2 - SCREEN_SIZE, TEXT_FONT_SIZE, LIGHTGRAY);
		}
	}
}

void DrawObstacles(const Rectangle* obstacles){
	for(int i = 0; i < NUM_OBSTACLES; ++i){
		DrawRectangleRec(obstacles[i], RED);
	}
}

void UpdatePlayerVertices(const Pose* pose,  Vector2* vertices){
	Vector2 topLeft = { 0 };
    Vector2 topRight = { 0 };
    Vector2 bottomLeft = { 0 };
    Vector2 bottomRight = { 0 };

    // Only calculate rotation if needed
    if (pose->theta == 0.0f)
    {
        float x = pose->x;
        float y = pose->y;
        topLeft = (Vector2){ x - REAL_PLAYER_SIZE_2, y + REAL_PLAYER_SIZE_2 };
        topRight = (Vector2){ x + REAL_PLAYER_SIZE, y + REAL_PLAYER_SIZE_2 };
        bottomLeft = (Vector2){ x - REAL_PLAYER_SIZE_2, y - REAL_PLAYER_SIZE };
        bottomRight = (Vector2){ x + REAL_PLAYER_SIZE, y - REAL_PLAYER_SIZE };
    }
    else
    {
        float sinRotation = sinf(pose->theta);
        float cosRotation = cosf(pose->theta);
        float x = pose->x;
        float y = pose->y;
        float dx = REAL_PLAYER_SIZE_2;
        float dy = REAL_PLAYER_SIZE_2;

		// Rotation by 0
        topRight.x = x + dx*cosRotation - dy*sinRotation;
        topRight.y = y + dx*sinRotation + dy*cosRotation;

		// Rotation by pi /2
        topLeft.x = x - dx*sinRotation - dy*cosRotation;
        topLeft.y = y + dx*cosRotation - dy*sinRotation;

        bottomLeft.x = x - dx*cosRotation + dy*sinRotation;
        bottomLeft.y = y - dx*sinRotation - dy*cosRotation;

        bottomRight.x = x + dx*sinRotation + dy*cosRotation;
        bottomRight.y = y - dx*cosRotation + dy*sinRotation;
    }

	vertices[0] = topLeft;
	vertices[1] = topRight;
	vertices[2] = bottomRight;
	vertices[3] = bottomLeft;
}

void UpdatePlayerState(Pose* playerState, const Vector2* command, Vector2* vertices, const float dt){


	// Differential Drive motion model
	playerState->x += (command->x * cosf(playerState->theta) * dt);
	playerState->y += (command->x * sinf(playerState->theta)* dt);
	playerState->theta += (command->y * dt); // Heading negated to properlly define the right hand rule

	UpdatePlayerVertices(playerState, vertices);

	// printf("Player: (x: %f y: %f theta: %f) \n", playerState->x, playerState->y, playerState->theta * RAD2DEG);
}

void DrawPlayer(const Pose* robotPosition, const Vector2* robotVelocity, const Vector2* vertices, bool hasCollided){

	Rectangle robot = {robotPosition->x * PIXEL_SCALE, robotPosition->y * PIXEL_SCALE, PLAYER_SIZE, PLAYER_SIZE};
	Vector2 origin = {robot.width/2.0f, robot.height / 2.0f};

	// DrawRectanglePro(
	// 	robot,
	// 	origin,
	// 	robotPosition->theta * RAD2DEG,
	// 	hasCollided? RED: BLACK
	// );

	Vector2 start = {robot.x, robot.y};
	Vector2 end = {robot.x +  PLAYER_SIZE /2.0f * cosf(robotPosition->theta), 
		           robot.y +  PLAYER_SIZE /2.0f * sinf(robotPosition->theta)};
	
	DrawLineEx(start, end, 2, GREEN);

	for(int i = 0; i < 4; ++i){
		DrawLineEx((Vector2){
			vertices[i].x * PIXEL_SCALE,
			vertices[i].y * PIXEL_SCALE,
		}, 
		(Vector2){
			vertices[(i + 1) % 4].x * PIXEL_SCALE,
			vertices[(i + 1) % 4].y * PIXEL_SCALE,
		}, 2, hasCollided? RED: BLUE);
	}
}

#if NUM_OBSTACLES > 0
void RandomPlayerStart(Pose* playerPosition, int occupancyGrid[][GRID_HEIGHT]){
		int rx, ry;

		do{
			rx = rand() % GRID_WIDTH; 
			ry = rand() % GRID_HEIGHT; 
		}while(occupancyGrid[rx][ry] != -1);

		playerPosition->x = rx * REAL_GRID_SIZE + REAL_GRID_SIZE_2;
		playerPosition->y = ry * REAL_GRID_SIZE + REAL_GRID_SIZE_2;


		float theta = (rand() % 4) * (PI / 2.0); // Robot starts at one of the 4 directions 

		playerPosition->theta = theta;
}
#else
void RandomPlayerStart(Pose* playerPosition){
		int	rx = rand() % GRID_WIDTH; 
		int	ry = rand() % GRID_HEIGHT; 
		
		playerPosition->x = rx * REAL_GRID_SIZE + REAL_GRID_SIZE_2;
		playerPosition->y = ry * REAL_GRID_SIZE + REAL_GRID_SIZE_2;

		float theta = (rand() % 4) * (PI / 2.0); // Robot starts at one of the 4 directions 

		playerPosition->theta = theta;
}
#endif

#define NUM_NEIGHBOR_CHECK 8
// 8 Connected neighbors corners
const GridVector2 neighbors[NUM_NEIGHBOR_CHECK] = {
	{1, 1},
	{1, -1},
	{-1, -1},
	{-1, 1},
	{0, 1},
	{1, 0},
	{-1, 0},
	{0, -1}
};

const bool InsideGrid(const GridVector2* gridCoordinate){
	return gridCoordinate->x >= 0 && gridCoordinate->x < GRID_WIDTH && gridCoordinate->y >= 0 && gridCoordinate->y < GRID_HEIGHT;
}

const bool OccupiesGrid(const GridVector2* gridCoordinate, const int occupancyGrid[][GRID_HEIGHT]){
	return InsideGrid(gridCoordinate) && occupancyGrid[gridCoordinate->x][gridCoordinate->y] != -1;
}

const bool GetObstacle(const GridVector2* gridCoordinate, const int occupancyGrid[][GRID_HEIGHT], const Vector2 gridCoordinates[][4], Vector2* coordinates){
	if(InsideGrid(gridCoordinate)){
		int coordinate = occupancyGrid[gridCoordinate->x][gridCoordinate->y];

		if(coordinate != -1){
			for(int i = 0; i < 4; ++i){
				coordinates[i] = gridCoordinates[coordinate][i];
			}
			
			return true;
		}
	}


	return false;
}


#define CLOSEST_SQUARE_THRESHOLD sqrtf(2) * REAL_GRID_SIZE_2

bool EntersSquare(const Vector2 startPos1,const Vector2 endPos1,const Vector2 squareCenter){
	const Vector2 ab = Vector2Subtract(endPos1, startPos1);
	const Vector2 ac = Vector2Subtract(squareCenter, startPos1);

	const float proj = Vector2DotProduct(ab, ac) / (REAL_PLAYER_SIZE * REAL_PLAYER_SIZE);

	if(proj < 0 || proj > 1){
		return false;
	}

	const Vector2 closestPoint = Vector2Add(startPos1,  Vector2MultiplyScalar(ab, proj)); 

	return Vector2Distance(closestPoint, squareCenter) <= CLOSEST_SQUARE_THRESHOLD;
}

static inline void ProjectOntoAxis(const Vector2 verts[4], float ux, float uy,
                                     float *min_out, float *max_out) {
    // Initial projection
    float proj = verts[0].x * ux + verts[0].y * uy;
    float min_p = proj;
    float max_p = proj;
    
    // Check remaining vertices
    for (int i = 1; i < 4; ++i) {
        proj = verts[i].x * ux + verts[i].y * uy;
        if (proj < min_p) min_p = proj;
        if (proj > max_p) max_p = proj;
    }
    *min_out = min_p;
    *max_out = max_p;
}


bool CheckPlayerCollision(const Pose* playerPosition, const Vector2* playerVertices, const int occupancyGrid[][GRID_HEIGHT], const Vector2 obstacleVertices[][4]) {
	GridVector2 playGrid = SnapToGridPose(playerPosition);

	// printf("[%f, %f]->[%d, %d]\n", playerPosition->x, playerPosition->y, playGrid.x, playGrid.y);

	if(OccupiesGrid(&playGrid, occupancyGrid)){
		return true;
	}

	 // Look at the 8 connected neighbors of choices instead of the whole grid ( simple speed up);
	for(int i = 0; i < 4; ++i){
		const GridVector2 vertexGrid = SnapToGridVec(&playerVertices[i]);
		if(OccupiesGrid(&vertexGrid, occupancyGrid)){
			return true;
		}
	}

	// Now only need to check if the the lines collide with the corder grids ( Only need to check) the corner edges ( it is assumed that the robot is less than or equal than the world obstacles sizes)

	Vector2 collisionPoint = {0};
	Vector2 obstacleCoordinates[4] = {0};

	const float c = cosf(playerPosition->theta);
    const float s = sinf(playerPosition->theta);
	const float axes[4][2] = {
        { 1.0f,  0.0f },
        { 0.0f,  1.0f },
        { -s,     c    },
        {  c,     s    }
    };

	// Separating Axis Theorem (SAT) for two convex polygons
	float playerMin[4], playerMax[4];
    for (int i = 0; i < 4; ++i) {
        float ux = axes[i][0], uy = axes[i][1];
        ProjectOntoAxis(playerVertices, ux, uy, &playerMin[i], &playerMax[i]);
    }

	for(int j = 0; j < NUM_NEIGHBOR_CHECK; ++j){
		const GridVector2 position = GridVector2Add(playGrid, neighbors[j]);

		// Check an obstacle even exists in that corner
		if(GetObstacle(&position, occupancyGrid, obstacleVertices, obstacleCoordinates)){
			bool separated = false;

			for (int i = 0; i < 4; ++i) {
				float ux = axes[i][0];
				float uy = axes[i][1];
				float minB, maxB;
                ProjectOntoAxis(obstacleCoordinates, ux, uy, &minB, &maxB);

				// If projections do not overlap, separating axis found
                if (playerMax[i] < minB || maxB < playerMin[i]) {
                    separated = true;
                    break;
                }
			}

			if (!separated) {
                return true;
            }
		}
	}

	return false;
}


int main ()
{
	const int screenWidth = SCREEN_SIZE;
	const int screenHeight = SCREEN_SIZE;

	
	Pose robotPosition = {(GRID_SIZE) / 2.0f, (GRID_SIZE)  / 2.0f, 0};
	Vector2 robotVertices[4] = {0};
	Vector2 robotVelocity = {0.0, 0.1};


	const Vector2 worldMap[] = {
		(Vector2){0, 0},
		(Vector2){REAL_GRID_SIZE* GRID_WIDTH, 0},
		(Vector2){REAL_GRID_SIZE* GRID_WIDTH, REAL_GRID_SIZE* GRID_HEIGHT},
		(Vector2){0, REAL_GRID_SIZE* GRID_HEIGHT},
	};

	#if NUM_OBSTACLES > 0
	
	// TODO Make these static global variables
	int occupancyGrid[GRID_WIDTH][GRID_HEIGHT];

	for(int i = 0; i < GRID_WIDTH; ++i){
		for(int j = 0; j < GRID_HEIGHT; ++j){
			occupancyGrid[i][j] = -1;
		}
	}

	
	// TODO Make these static global variables
	int obstacleLocations [NUM_OBSTACLES][2] = {0}; // Grid x, y locations of each of the obstacles
	Rectangle obstacleCollisions [NUM_OBSTACLES];
	Vector2 obstacleVerticles[NUM_OBSTACLES][4] = {0};

	// srand(time(NULL));   // Initialization of random seed, should only be called once.
	srand(0);

	GenerateObstacles(occupancyGrid, obstacleLocations, obstacleCollisions, obstacleVerticles);

	RandomPlayerStart(&robotPosition, occupancyGrid);
	#else
	RandomPlayerStart(&robotPosition);
	#endif 

	#ifdef HAS_LIDAR
	// TODO Make these static global variables

	float lidarScan[NUM_LIDAR_SCANS] = {0.0f};
	Vector2 lidarHits[NUM_LIDAR_SCANS] = {0};
	#endif


	printf("Player Starting location (%f, %f, %f)", robotPosition.x, robotPosition.y, robotPosition.theta * RAD2DEG);

	// Tell the window to use vsync and work on high DPI displays
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT | FLAG_WINDOW_TRANSPARENT);

	Camera2D camera = { 0 };

	// Center camera to middle of the screen
	camera.target.x =  (GRID_WIDTH * GRID_SIZE -SCREEN_SIZE) / 2.0f;
	camera.target.y =  -(GRID_HEIGHT * GRID_SIZE + SCREEN_SIZE) / 2.0f;

    camera.zoom = 1.0f; // To add some padding zoom out initially

	int zoomMode = 0;   // 0-Mouse Wheel, 1-Mouse Move

	// Create the window and OpenGL context
	InitWindow(screenWidth, screenHeight, "ME 423 - Robotics  Simulation");

	// Utility function from resource_dir.h to find the resources folder and set it as the current working directory so we can load from it
	// SearchAndSetResourceDir("resources");

	SetTargetFPS(60);

	bool hasCollided = false;

	float dt = 0.1;
	
	// game loop
	while (!WindowShouldClose())		// run the loop untill the user presses ESCAPE or presses the Close button on the window
	{
		if (IsKeyPressed(KEY_R)){
			#if NUM_OBSTACLES > 0
			RandomPlayerStart(&robotPosition, occupancyGrid);
			#else
			RandomPlayerStart(&robotPosition);
			#endif 
		}

		// Update
		//----------------------------------------------------------------------------------
		CameraLogic(&camera, &zoomMode);

		UpdatePlayerState(&robotPosition, &robotVelocity, robotVertices, dt);

		#if NUM_OBSTACLES > 0
		hasCollided = CheckPlayerCollision(&robotPosition, robotVertices, occupancyGrid, obstacleVerticles);
		#endif 

		#ifdef HAS_LIDAR
		#if NUM_OBSTACLES > 0
		UpdateLidarScan(&robotPosition, lidarScan, lidarHits, worldMap, occupancyGrid, obstacleVerticles);
		#else
		UpdateLidarScan(&robotPosition, lidarScan, worldMap);
		#endif
		#endif

		//----------------------------------------------------------------------------------

		// Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode2D(camera);
                rlPushMatrix();
				// Since inverting y axis we need to change the culling as well otherwise things don't get rendered
				rlSetCullFace(RL_CULL_FACE_FRONT);
				// Make it proper cartesian coordinate system plotting
                rlScalef(1.0, -1.0, 1.0);

				#if NUM_OBSTACLES > 0
					// If the map has obstacles then draw them
					DrawObstacles(obstacleCollisions);
				#endif	

				#ifdef HAS_LIDAR 
					DrawLiDAR(&robotPosition, lidarScan, lidarHits);
				#endif
				// Draw player
				DrawPlayer(&robotPosition, &robotVelocity, robotVertices, hasCollided);

				// Draw the grid world
				DrawWorldGrid(screenWidth, screenHeight);

				rlPopMatrix();
            EndMode2D();

			rlSetCullFace(RL_CULL_FACE_BACK);

			BeginMode2D(camera);
				// Need to plot seperately otherwise the text will be plotted inverted
                DrawWorldGridText();
            EndMode2D();

			GuiSliderBar((Rectangle){ 600, 40, 120, 20 }, "dt", TextFormat("%.2f", dt), &dt, -0.2, 0.2);
			GuiSliderBar((Rectangle){ 600, 80, 120, 20 }, "v", TextFormat("%.2f", robotVelocity.x), &robotVelocity.x, -0.2, 0.2);
			GuiSliderBar((Rectangle){ 600, 120, 120, 20 }, "w", TextFormat("%.2f", robotVelocity.y), &robotVelocity.y, -0.2, 0.2);
            
            // Draw mouse reference
            //Vector2 mousePos = GetWorldToScreen2D(GetMousePosition(), camera);
            DrawCircleV(GetMousePosition(), 4, DARKGRAY);

			//TODO Make the coordinate be based on real coordinate position on camera position
            DrawTextEx(GetFontDefault(), TextFormat("[%i, %i]", GetMouseX(), GetMouseY()),  
                Vector2Add(GetMousePosition(), (Vector2){ -44, -24 }), 20, 2, BLACK);

            DrawText("[1][2] Select mouse zoom mode (Wheel or Move)", 20, 20, 20, DARKGRAY);
            if (zoomMode == 0) DrawText("Mouse left button drag to move, mouse wheel to zoom", 20, 50, 20, DARKGRAY);
            else DrawText("Mouse left button drag to move, mouse press and move to zoom", 20, 50, 20, DARKGRAY);
        
        EndDrawing();
        //----------------------------------------------------------------------------------

	}

	// cleanup

	// destroy the window and cleanup the OpenGL context
	CloseWindow();
	return 0;
}
