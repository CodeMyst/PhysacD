import bindbc.raylib;
import physac;

alias RVec2 = bindbc.raylib.Vector2;
alias PVec2 = physac.Vector2;

int main()
{
    loadRaylib();

    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 800;
    int screenHeight = 450;

    SetConfigFlags(ConfigFlags.FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "[physac] Basic demo");

    // Physac logo drawing position
    int logoX = screenWidth - MeasureText("Physac", 30) - 10;
    int logoY = 15;

    // Initialize physics and default physics bodies
    initPhysics();

    // Create floor rectangle physics body
    PhysicsBody floor = createPhysicsBodyRectangle(physac.Vector2(screenWidth/2, screenHeight), 500f, 100f, 10f);
    floor.enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    // Create obstacle circle physics body
    PhysicsBody circle = createPhysicsBodyCircle(physac.Vector2(screenWidth/2, screenHeight/2), 45f, 10f);
    circle.enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    
    SetTargetFPS(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        // Physics body creation inputs
        if (IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_LEFT))
        {
            RVec2 rv = GetMousePosition();
            createPhysicsBodyPolygon(PVec2(rv.x, rv.y), cast(float) GetRandomValue(20, 80), GetRandomValue(3, 8), 10f);
        }
        else if (IsMouseButtonPressed(MouseButton.MOUSE_BUTTON_RIGHT))
        {
            RVec2 rv = GetMousePosition();
            createPhysicsBodyCircle(PVec2(rv.x, rv.y), cast(float) GetRandomValue(10, 45), 10);
        }

        // Destroy falling physics bodies
        int bodiesCount = getPhysicsBodiesCount();
        for (int i = bodiesCount - 1; i >= 0; i--)
        {
            PhysicsBody body = getPhysicsBody(i);
            
            if ((body != null) && (body.position.y > screenHeight*2))
                destroyPhysicsBody(body);
        }
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            DrawFPS(screenWidth - 90, screenHeight - 30);

            // Draw created physics bodies
            bodiesCount = getPhysicsBodiesCount();
            for (int i = 0; i < bodiesCount; i++)
            {
                PhysicsBody body = getPhysicsBody(i);

                if (body != null)
                {
                    int vertexCount = getPhysicsShapeVerticesCount(i);
                    for (int j = 0; j < vertexCount; j++)
                    {
                        // Get physics bodies shape vertices to draw lines
                        // Note: GetPhysicsShapeVertex() already calculates rotation transformations
                        physac.Vector2 vertexA = getPhysicsShapeVertex(body, j);

                        int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
                        physac.Vector2 vertexB = getPhysicsShapeVertex(body, jj);

                        RVec2 va = RVec2(vertexA.x, vertexA.y);
                        RVec2 vb = RVec2(vertexB.x, vertexB.y);
                        DrawLineV(va, vb, GREEN);     // Draw a line between two vertex positions
                    }
                }
            }

            DrawText("Left mouse button to create a polygon", 10, 10, 10, WHITE);
            DrawText("Right mouse button to create a circle", 10, 25, 10, WHITE);

            DrawText("Physac", logoX, logoY, 30, WHITE);
            DrawText("Powered by", logoX + 50, logoY - 7, 10, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------   
    closePhysics();       // Unitialize physics
    
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
