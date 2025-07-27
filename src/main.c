/*
Raylib example file.
This is an example main file for a simple raylib project.
Use this as a starting point or replace it with your code.

by Jeffery Myers is marked with CC0 1.0. To view a copy of this license, visit https://creativecommons.org/publicdomain/zero/1.0/

*/

#include "raylib.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "rlgl.h"
#include "raymath.h"

#include "resource_dir.h"	// utility header for SearchAndSetResourceDir
#include <math.h>

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

struct Pose {
	float x;
	float y;
	float theta;
};

#define NUM_OBSTACLES 20

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
#define NUM_LIDAR_SCANS 10 // Numer of datapoints inside the LiDAR scan
#define LIDAR_ANGULAR_RESOLUTION 2 * PI / NUM_LIDAR_SCANS // Angular resolution of LiDAR in radians 
#endif


void GenerateObstacles(bool occupancyGrid[][GRID_HEIGHT], int obtacleLocations[][2], Rectangle * obstacleCollisions) {
	for(int i= 0; i < NUM_OBSTACLES; ++i){
		int rx, ry;

		do{
			rx = rand() % GRID_WIDTH; 
			ry = rand() % GRID_HEIGHT; 
		}while(occupancyGrid[rx][ry]);

		occupancyGrid[rx][ry] = true;

		obtacleLocations[i][0] = rx;      // Returns a pseudo-random integer between 0 and gridWidth.
		obtacleLocations[i][1] = ry; // Returns a pseudo-random integer between 0 and gridHeight.

		obstacleCollisions[i].x = (float) rx * GRID_SIZE;
		obstacleCollisions[i].y = (float) ry * GRID_SIZE;
		obstacleCollisions[i].width = (float) GRID_SIZE;
		obstacleCollisions[i].height = (float) GRID_SIZE;

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

void UpdateLidarScan(const struct Pose *robotPosition, float* lidarScan, const Rectangle* obstacleCollisions, const Rectangle* worldBorder){
	//TODO Have Ray Casting Performed such that it gets the closest hit point to an obstacle and populates the lidarScan array

}

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

#define DT 0.1

#define REAL_GRID_SIZE 0.3 // Represents the true size of a grid cell in meters

#define PIXEL_SCALE  GRID_SIZE / REAL_GRID_SIZE // Represents the gridsize to pixel scaling for the motion controlling in meters

void UpdatePlayerState(struct Pose* playerState, const Vector2* command){


	// Differential Drive motion model
	playerState->x += (command->x * cosf(playerState->theta) * DT);
	playerState->y += (command->x * sinf(playerState->theta)* DT);
	playerState->theta += (command->y * DT); // Heading negated to properlly define the right hand rule

	// printf("Player: (x: %f y: %f theta: %f) \n", playerState->x, playerState->y, playerState->theta * RAD2DEG);
}

#define REAL_PLAYER_SIZE 0.2 // Dimension of the robot in meters
#define PLAYER_SIZE REAL_PLAYER_SIZE * PIXEL_SCALE // Size of the robot in pixel size

void DrawPlayer(const struct Pose* robotPosition, const Vector2* robotVelocity){

	Rectangle robot = {robotPosition->x * PIXEL_SCALE, robotPosition->y * PIXEL_SCALE, PLAYER_SIZE, PLAYER_SIZE};
	Vector2 origin = {robot.width/2.0f, robot.height / 2.0f};

	DrawRectanglePro(
		robot,
		origin,
		robotPosition->theta * RAD2DEG,
		BLACK
	);

	Vector2 start = {robot.x, robot.y};
	Vector2 end = {robot.x +  PLAYER_SIZE /2.0f * cosf(robotPosition->theta), 
		           robot.y +  PLAYER_SIZE /2.0f * sinf(robotPosition->theta)};
	
	DrawLineEx(start, end, 2, GREEN);
}

void RandomPlayerStart(struct Pose* playerPosition, bool occupancyGrid[][GRID_HEIGHT]){
		int rx, ry;

		do{
			rx = rand() % GRID_WIDTH; 
			ry = rand() % GRID_HEIGHT; 
		}while(occupancyGrid[rx][ry]);

		playerPosition->x = rx * REAL_GRID_SIZE + REAL_GRID_SIZE /2.0f;
		playerPosition->y = ry * REAL_GRID_SIZE + REAL_GRID_SIZE /2.0f;


		float theta = (rand() % 4) * (PI / 2.0); // Robot starts at one of the 4 directions 

		playerPosition->theta = theta;
}


int main ()
{
	const int screenWidth = SCREEN_SIZE;
	const int screenHeight = SCREEN_SIZE;


	const Rectangle worldMap = {0, 0, GRID_SIZE * GRID_WIDTH, GRID_SIZE * GRID_HEIGHT};

	bool occupancyGrid[GRID_WIDTH][GRID_HEIGHT] = {false};

	#if NUM_OBSTACLES > 0
	int obstacleLocations [NUM_OBSTACLES][2] = {0}; // Grid x, y locations of each of the obstacles
	Rectangle obstacleCollisions [NUM_OBSTACLES];

	srand(time(NULL));   // Initialization of random seed, should only be called once.

	GenerateObstacles(occupancyGrid, obstacleLocations, obstacleCollisions);
	#endif 

	#ifdef HAS_LIDAR
	float lidarScan[NUM_LIDAR_SCANS] = {0.0};
	#endif

	struct Pose robotPosition = {(GRID_SIZE) / 2.0f, (GRID_SIZE)  / 2.0f, 0};
	Vector2 robotVelocity = {0.0, 0.0};

	RandomPlayerStart(&robotPosition, occupancyGrid);

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
	
	// game loop
	while (!WindowShouldClose())		// run the loop untill the user presses ESCAPE or presses the Close button on the window
	{
		if (IsKeyPressed(KEY_R)){
			RandomPlayerStart(&robotPosition, occupancyGrid);
		}

		// Update
		//----------------------------------------------------------------------------------
		CameraLogic(&camera, &zoomMode);

		UpdatePlayerState(&robotPosition, &robotVelocity);

		#ifdef HAS_LIDAR
		UpdateLidarScan(&robotPosition, lidarScan, obstacleCollisions, &worldMap);
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

				// Draw player
				DrawPlayer(&robotPosition, &robotVelocity);

				// Draw the grid world
				DrawWorldGrid(screenWidth, screenHeight);

				rlPopMatrix();
            EndMode2D();

			rlSetCullFace(RL_CULL_FACE_BACK);

			BeginMode2D(camera);
				// Need to plot seperately otherwise the text will be plotted inverted
                DrawWorldGridText();
            EndMode2D();
            
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
