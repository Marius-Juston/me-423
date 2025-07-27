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

#define NUM_OBSTACLES 10

#define GRID_WIDTH 10
#define GRID_HEIGHT 20
#define SCREEN_SIZE 800
#define SPACING 10

// If you want to manually specify the grid spacing for vizualization uncomment this line
// #define GRID_SIZE 20

#ifndef GRID_SIZE
// Will find the grid size necessary to have everything plotted within the specific screen size, with padding
#define GRID_SIZE (SCREEN_SIZE - 2 * SPACING) / max(GRID_HEIGHT, GRID_WIDTH)
#endif


void GenerateObstacles(int* obtacleLocations) {
	for(int i= 0; i < NUM_OBSTACLES; ++i){
		int rx = rand() % GRID_WIDTH; 
		int ry = rand() % GRID_HEIGHT; 

		obtacleLocations[i * 2] = rx;      // Returns a pseudo-random integer between 0 and gridWidth.
		obtacleLocations[i * 2 + 1] = ry; // Returns a pseudo-random integer between 0 and gridHeight.

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

#define TEXT_OFFSET 1
#define TEXT_FONT_SIZE 10

void DrawWorldGrid(){
	// Draw centered world grid

	for (int i = 0; i < GRID_WIDTH + 1; ++i)
	{
		DrawLineV((Vector2){(float)(GRID_SIZE *i + SPACING), SPACING}, (Vector2){ (float)(GRID_SIZE*i + SPACING), (float) (GRID_SIZE * (GRID_HEIGHT ) + SPACING)}, LIGHTGRAY);
	}

	for (int i = 0; i < GRID_HEIGHT + 1; ++i)
	{
		DrawLineV((Vector2){SPACING, (float)(GRID_SIZE*i + SPACING)}, (Vector2){ (float)(GRID_SIZE * (GRID_WIDTH )  + SPACING), (float)(GRID_SIZE *i + SPACING)}, LIGHTGRAY);
	}


	for (int r = 0; r < GRID_HEIGHT; r++)
	{
		for (int c = 0; c < GRID_WIDTH; ++c)
		{
			// TODO figure out how to have font size be dynamically set based on the GRID_SIZE variable to ensure that it all fits inside it
			DrawText(TextFormat("[%i,%i]", c, r), GRID_SIZE*c + 12,  GRID_SIZE*r + 12, TEXT_FONT_SIZE, LIGHTGRAY);
		}
	}
}

void DrawObstacles(const int* obstacles){
	for(int i = 0; i < NUM_OBSTACLES; ++i){
		int x = obstacles[i  * 2] * GRID_SIZE + SPACING;
		int y = obstacles[i  * 2 + 1] * GRID_SIZE + SPACING;

		DrawRectangle(x, y, GRID_SIZE, GRID_SIZE, RED);
	}
}


int main ()
{
	const int screenWidth = SCREEN_SIZE;
	const int screenHeight = SCREEN_SIZE;


	#if NUM_OBSTACLES > 0
	int obstacleLocations [NUM_OBSTACLES * 2] = {0}; // Grid x, y locations of each of the obstacles

	srand(time(NULL));   // Initialization of random seed, should only be called once.

	GenerateObstacles(obstacleLocations);
	#endif 

	Vector2 robotPosition = {0, 0};

	// Tell the window to use vsync and work on high DPI displays
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT | FLAG_WINDOW_TRANSPARENT);

	Camera2D camera = { 0 };
    camera.zoom = 1.0f;
	int zoomMode = 0;   // 0-Mouse Wheel, 1-Mouse Move

	// Create the window and OpenGL context
	InitWindow(screenWidth, screenHeight, "ME 423 - Robotics  Simulation");

	// Utility function from resource_dir.h to find the resources folder and set it as the current working directory so we can load from it
	// SearchAndSetResourceDir("resources");

	SetTargetFPS(60);
	
	// game loop
	while (!WindowShouldClose())		// run the loop untill the user presses ESCAPE or presses the Close button on the window
	{
		// Update
		//----------------------------------------------------------------------------------
		CameraLogic(&camera, &zoomMode);

		//----------------------------------------------------------------------------------

		// Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode2D(camera);

				#if NUM_OBSTACLES > 0
					DrawObstacles(obstacleLocations);
				#endif	

                // Draw the 3d grid, rotated 90 degrees and centered around 0,0 
                // just so we have something in the XY plane
				DrawWorldGrid(screenWidth, screenHeight);
                // Draw a reference circle
                
            EndMode2D();
            
            // Draw mouse reference
            //Vector2 mousePos = GetWorldToScreen2D(GetMousePosition(), camera)
            DrawCircleV(GetMousePosition(), 4, DARKGRAY);
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
