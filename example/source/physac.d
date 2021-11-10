/**********************************************************************************************
*
*   PhysacD - Port of Physac for the D programming language.
*       Author: CodeMyst
*
*   Below is the original notice of Physac:
*
*   Physac v1.1 - 2D Physics library for videogames
*
*   DESCRIPTION:
*
*   Physac is a small 2D physics library written in pure C. The engine uses a fixed time-step thread loop
*   to simluate physics. A physics step contains the following phases: get collision information,
*   apply dynamics, collision solving and position correction. It uses a very simple struct for physic
*   bodies with a position vector to be used in any 3D rendering API.
*
*   CONFIGURATION:
*
*   #define PHYSAC_IMPLEMENTATION
*       Generates the implementation of the library into the included file.
*       If not defined, the library is in header only mode and can be included in other headers
*       or source files without problems. But only ONE file should hold the implementation.
*
*   #define PHYSAC_STATIC (defined by default)
*       The generated implementation will stay private inside implementation file and all
*       internal symbols and functions will only be visible inside that file.
*
*   #define PHYSAC_NO_THREADS
*       The generated implementation won't include pthread library and user must create a secondary thread to call PhysicsThread().
*       It is so important that the thread where PhysicsThread() is called must not have v-sync or any other CPU limitation.
*
*   #define PHYSAC_STANDALONE
*       Avoid raylib.h header inclusion in this file. Data types defined on raylib are defined
*       internally in the library and input management and drawing functions must be provided by
*       the user (check library implementation for further details).
*
*   #define PHYSAC_DEBUG
*       Traces log messages when creating and destroying physics bodies and detects errors in physics
*       calculations and reference exceptions; it is useful for debug purposes
*
*   #define PHYSAC_MALLOC()
*   #define PHYSAC_FREE()
*       You can define your own malloc/free implementation replacing stdlib.h malloc()/free() functions.
*       Otherwise it will include stdlib.h and use the C standard library malloc()/free() function.
*
*
*   NOTE 1: Physac requires multi-threading, when InitPhysics() a second thread is created to manage physics calculations.
*   NOTE 2: Physac requires static C library linkage to avoid dependency on MinGW DLL (-static -lpthread)
*
*   Use the following code to compile:
*   gcc -o $(NAME_PART).exe $(FILE_NAME) -s -static -lraylib -lpthread -lopengl32 -lgdi32 -lwinmm -std=c99
*
*   VERY THANKS TO:
*       - raysan5: helped with library design
*       - ficoos: added support for Linux
*       - R8D8: added support for Linux
*       - jubalh: fixed implementation of time calculations
*       - a3f: fixed implementation of time calculations
*       - define-private-public: added support for OSX
*       - pamarcos: fixed implementation of physics steps
*       - noshbar: fixed some memory leaks
*
*
*   LICENSE: zlib/libpng
*
*   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

import core.stdc.stdlib;
import core.stdc.stdio;
import core.stdc.math;
import core.sys.posix.pthread;
import core.time;
import std.algorithm;

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
enum int    PHYSAC_MAX_BODIES               = 64;
enum int    PHYSAC_MAX_MANIFOLDS            = 4096;
enum int    PHYSAC_MAX_VERTICES             = 24;
enum int    PHYSAC_CIRCLE_VERTICES          = 24;

enum int    PHYSAC_COLLISION_ITERATIONS     = 100;
enum double PHYSAC_PENETRATION_ALLOWANCE    = 0.05f;
enum double PHYSAC_PENETRATION_CORRECTION   = 0.4f;

enum double PHYSAC_PI                       = 3.14159265358979323846;
enum double PHYSAC_DEG2RAD                  = (PHYSAC_PI/180.0f);

void*       PHYSAC_MALLOC(size_t size)      { return malloc(size); }
void        PHYSAC_FREE(void* ptr)          { free(ptr); }

//----------------------------------------------------------------------------------
// Types and Structures Definition
// NOTE: Below types are required for PHYSAC_STANDALONE usage
//----------------------------------------------------------------------------------
alias PhysicsBody = PhysicsBodyData*;
alias PhysicsManifold = PhysicsManifoldData*;

// Vector2 type
struct Vector2 {
    float x;
    float y;
}

enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON }

// Mat2 type (used for polygon shape rotation matrix)
struct Mat2 {
    float m00;
    float m01;
    float m10;
    float m11;
}

struct PolygonData {
    uint vertexCount;                           // Current used vertex and normals count
    Vector2[PHYSAC_MAX_VERTICES] positions;     // Polygon vertex positions vectors
    Vector2[PHYSAC_MAX_VERTICES] normals;       // Polygon vertex normals vectors
}

struct PhysicsShape {
    PhysicsShapeType type;                      // Physics shape type (circle or polygon)
    PhysicsBody body;                           // Shape physics body reference
    float radius;                               // Circle shape radius (used for circle shapes)
    Mat2 transform;                             // Vertices transform matrix 2x2
    PolygonData vertexData;                     // Polygon shape vertices position and normals data (just used for polygon shapes)
}

struct PhysicsBodyData {
    uint id;                                    // Reference unique identifier
    bool enabled;                               // Enabled dynamics state (collisions are calculated anyway)
    Vector2 position;                           // Physics body shape pivot
    Vector2 velocity;                           // Current linear velocity applied to position
    Vector2 force;                              // Current linear force (reset to 0 every step)
    float angularVelocity;                      // Current angular velocity applied to orient
    float torque;                               // Current angular force (reset to 0 every step)
    float orient;                               // Rotation in radians
    float inertia;                              // Moment of inertia
    float inverseInertia;                       // Inverse value of inertia
    float mass;                                 // Physics body mass
    float inverseMass;                          // Inverse value of mass
    float staticFriction;                       // Friction when the body has not movement (0 to 1)
    float dynamicFriction;                      // Friction when the body has movement (0 to 1)
    float restitution;                          // Restitution coefficient of the body (0 to 1)
    bool useGravity;                            // Apply gravity force to dynamics
    bool isGrounded;                            // Physics grounded on other body state
    bool freezeOrient;                          // Physics rotation constraint
    PhysicsShape shape;                         // Physics body shape information (type, radius, vertices, normals)
}

struct PhysicsManifoldData {
    uint id;                                    // Reference unique identifier
    PhysicsBody bodyA;                          // Manifold first physics body reference
    PhysicsBody bodyB;                          // Manifold second physics body reference
    float penetration;                          // Depth of penetration from collision
    Vector2 normal;                             // Normal direction vector from 'a' to 'b'
    Vector2[2] contacts;                        // Points of contact during collision
    uint contactsCount;                         // Current collision number of contacts
    float restitution;                          // Mixed restitution during collision
    float dynamicFriction;                      // Mixed dynamic friction during collision
    float staticFriction;                       // Mixed static friction during collision
}

/***********************************************************************************
*
*   PHYSAC IMPLEMENTATION
*
************************************************************************************/

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
double      PHYSAC_FLT_MAX              = 3.402823466e+38f;
double      PHYSAC_EPSILON              = 0.000001f;
double      PHYSAC_K                    = 1.0f/3.0f;
Vector2     PHYSAC_VECTOR_ZERO          = Vector2(0, 0);

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
private __gshared pthread_t physicsThreadId;                           // Physics thread id
private __gshared uint usedMemory = 0;                                 // Total allocated dynamic memory
private __gshared bool physicsThreadEnabled = false;                   // Physics thread enabled state
private __gshared double baseTime = 0.0;                               // Offset time for MONOTONIC clock
private __gshared double startTime = 0.0;                              // Start time in milliseconds
private __gshared double deltaTime = 1.0/60.0/10.0 * 1000;             // Delta time used for physics steps, in milliseconds
private __gshared double currentTime = 0.0;                            // Current time in milliseconds
private __gshared ulong frequency = 0;                              // Hi-res clock frequency

private __gshared double accumulator = 0.0;                            // Physics time step delta time accumulator
private __gshared uint stepsCount = 0;                                 // Total physics steps processed
private __gshared Vector2 gravityForce = { 0.0f, 9.81f };              // Physics world gravity force
private __gshared PhysicsBody[PHYSAC_MAX_BODIES] bodies;               // Physics bodies pointers array
private __gshared uint physicsBodiesCount = 0;                         // Physics world current bodies counter
private __gshared PhysicsManifold[PHYSAC_MAX_MANIFOLDS] contacts;      // Physics bodies pointers array
private __gshared uint physicsManifoldsCount = 0;                      // Physics world current manifolds counter

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
// Initializes physics values, pointers and creates physics loop thread
void initPhysics()
{
    pthread_create(&physicsThreadId, null, &physicsLoop, null);

    // Initialize high resolution timer
    initTimer();

    debug
    {
        printf("[PHYSAC] physics module initialized successfully\n");
    }

    accumulator = 0.0;
}

// Returns true if physics thread is currently enabled
bool isPhysicsEnabled()
{
    return physicsThreadEnabled;
}

// Sets physics global gravity force
void setPhysicsGravity(float x, float y)
{
    gravityForce.x = x;
    gravityForce.y = y;
}

// Creates a new circle physics body with generic parameters
PhysicsBody createPhysicsBodyCircle(Vector2 pos, float radius, float density)
{
    PhysicsBody newBody = cast(PhysicsBody) PHYSAC_MALLOC(PhysicsBodyData.sizeof);
    usedMemory += PhysicsBodyData.sizeof;

    int newId = findAvailableBodyIndex();
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody.id = newId;
        newBody.enabled = true;
        newBody.position = pos;
        newBody.velocity = PHYSAC_VECTOR_ZERO;
        newBody.force = PHYSAC_VECTOR_ZERO;
        newBody.angularVelocity = 0.0f;
        newBody.torque = 0.0f;
        newBody.orient = 0.0f;
        newBody.shape.type = PhysicsShapeType.PHYSICS_CIRCLE;
        newBody.shape.body = newBody;
        newBody.shape.radius = radius;
        newBody.shape.transform = mat2Radians(0.0f);
        newBody.shape.vertexData = PolygonData(0, Vector2(0, 0), Vector2(0, 0));

        newBody.mass = PHYSAC_PI*radius*radius*density;
        newBody.inverseMass = ((newBody.mass != 0.0f) ? 1.0f/newBody.mass : 0.0f);
        newBody.inertia = newBody.mass*radius*radius;
        newBody.inverseInertia = ((newBody.inertia != 0.0f) ? 1.0f/newBody.inertia : 0.0f);
        newBody.staticFriction = 0.4f;
        newBody.dynamicFriction = 0.2f;
        newBody.restitution = 0.0f;
        newBody.useGravity = true;
        newBody.isGrounded = false;
        newBody.freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;

        debug
        {
            printf("[PHYSAC] created polygon physics body id %i\n", newBody.id);
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] new physics body creation failed because there is any available id to use\n");
        }
    }

    return newBody;
}

// Creates a new rectangle physics body with generic parameters
PhysicsBody createPhysicsBodyRectangle(Vector2 pos, float width, float height, float density)
{
    PhysicsBody newBody = cast(PhysicsBody) PHYSAC_MALLOC(PhysicsBodyData.sizeof);
    usedMemory += PhysicsBodyData.sizeof;

    int newId = findAvailableBodyIndex();
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody.id = newId;
        newBody.enabled = true;
        newBody.position = pos;
        newBody.velocity = Vector2(0, 0);
        newBody.force = Vector2(0, 0);
        newBody.angularVelocity = 0.0f;
        newBody.torque = 0.0f;
        newBody.orient = 0.0f;
        newBody.shape.type = PhysicsShapeType.PHYSICS_POLYGON;
        newBody.shape.body = newBody;
        newBody.shape.radius = 0.0f;
        newBody.shape.transform = mat2Radians(0.0f);
        newBody.shape.vertexData = createRectanglePolygon(pos, Vector2(width, height));

        // Calculate centroid and moment of inertia
        Vector2 center = Vector2(0.0f, 0.0f);
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody.shape.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody.shape.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody.shape.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody.shape.vertexData.positions[nextIndex];

            float D = mathCrossVector2(p1, p2);
            float triangleArea = D/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*PHYSAC_K*(p1.x + p2.x);
            center.y += triangleArea*PHYSAC_K*(p1.y + p2.y);

            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*PHYSAC_K*D)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody.shape.vertexData.vertexCount; i++)
        {
            newBody.shape.vertexData.positions[i].x -= center.x;
            newBody.shape.vertexData.positions[i].y -= center.y;
        }

        newBody.mass = density*area;
        newBody.inverseMass = ((newBody.mass != 0.0f) ? 1.0f/newBody.mass : 0.0f);
        newBody.inertia = density*inertia;
        newBody.inverseInertia = ((newBody.inertia != 0.0f) ? 1.0f/newBody.inertia : 0.0f);
        newBody.staticFriction = 0.4f;
        newBody.dynamicFriction = 0.2f;
        newBody.restitution = 0.0f;
        newBody.useGravity = true;
        newBody.isGrounded = false;
        newBody.freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;

        debug
        {
            printf("[PHYSAC] created polygon physics body id %i\n", newBody.id);
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] new physics body creation failed because there is any available id to use\n");
        }
    }

    return newBody;
}

// Creates a new polygon physics body with generic parameters
PhysicsBody createPhysicsBodyPolygon(Vector2 pos, float radius, int sides, float density)
{
    PhysicsBody newBody = cast(PhysicsBody) PHYSAC_MALLOC(PhysicsBodyData.sizeof);
    usedMemory += PhysicsBodyData.sizeof;

    int newId = findAvailableBodyIndex();
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody.id = newId;
        newBody.enabled = true;
        newBody.position = pos;
        newBody.velocity = PHYSAC_VECTOR_ZERO;
        newBody.force = PHYSAC_VECTOR_ZERO;
        newBody.angularVelocity = 0.0f;
        newBody.torque = 0.0f;
        newBody.orient = 0.0f;
        newBody.shape.type = PhysicsShapeType.PHYSICS_POLYGON;
        newBody.shape.body = newBody;
        newBody.shape.transform = mat2Radians(0.0f);
        newBody.shape.vertexData = createRandomPolygon(radius, sides);

        // Calculate centroid and moment of inertia
        Vector2 center = Vector2(0, 0);
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody.shape.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 position1 = newBody.shape.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody.shape.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 position2 = newBody.shape.vertexData.positions[nextIndex];

            float cross = mathCrossVector2(position1, position2);
            float triangleArea = cross/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*PHYSAC_K*(position1.x + position2.x);
            center.y += triangleArea*PHYSAC_K*(position1.y + position2.y);

            float intx2 = position1.x*position1.x + position2.x*position1.x + position2.x*position2.x;
            float inty2 = position1.y*position1.y + position2.y*position1.y + position2.y*position2.y;
            inertia += (0.25f*PHYSAC_K*cross)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody.shape.vertexData.vertexCount; i++)
        {
            newBody.shape.vertexData.positions[i].x -= center.x;
            newBody.shape.vertexData.positions[i].y -= center.y;
        }

        newBody.mass = density*area;
        newBody.inverseMass = ((newBody.mass != 0.0f) ? 1.0f/newBody.mass : 0.0f);
        newBody.inertia = density*inertia;
        newBody.inverseInertia = ((newBody.inertia != 0.0f) ? 1.0f/newBody.inertia : 0.0f);
        newBody.staticFriction = 0.4f;
        newBody.dynamicFriction = 0.2f;
        newBody.restitution = 0.0f;
        newBody.useGravity = true;
        newBody.isGrounded = false;
        newBody.freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;

        debug
        {
            printf("[PHYSAC] created polygon physics body id %i\n", newBody.id);
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] new physics body creation failed because there is any available id to use\n");
        }
    }

    return newBody;
}

// Adds a force to a physics body
void physicsAddForce(PhysicsBody body, Vector2 force)
{
    if (body != null)
        body.force = vector2Add(body.force, force);
}

// Adds an angular force to a physics body
void physicsAddTorque(PhysicsBody body, float amount)
{
    if (body != null)
        body.torque += amount;
}

// Shatters a polygon shape physics body to little physics bodies with explosion force
void physicsShatter(PhysicsBody body, Vector2 position, float force)
{
    if (body != null)
    {
        if (body.shape.type == PhysicsShapeType.PHYSICS_POLYGON)
        {
            PolygonData vertexData = body.shape.vertexData;
            bool collision = false;

            for (int i = 0; i < vertexData.vertexCount; i++)
            {
                Vector2 positionA = body.position;
                Vector2 positionB = mat2MultiplyVector2(body.shape.transform,
                        vector2Add(body.position, vertexData.positions[i]));
                int nextIndex = (((i + 1) < vertexData.vertexCount) ? (i + 1) : 0);
                Vector2 positionC = mat2MultiplyVector2(body.shape.transform,
                        vector2Add(body.position, vertexData.positions[nextIndex]));

                // Check collision between each triangle
                float alpha = ((positionB.y - positionC.y)*(position.x - positionC.x) +
                               (positionC.x - positionB.x)*(position.y - positionC.y))/
                              ((positionB.y - positionC.y)*(positionA.x - positionC.x) +
                               (positionC.x - positionB.x)*(positionA.y - positionC.y));

                float beta = ((positionC.y - positionA.y)*(position.x - positionC.x) +
                              (positionA.x - positionC.x)*(position.y - positionC.y))/
                             ((positionB.y - positionC.y)*(positionA.x - positionC.x) +
                              (positionC.x - positionB.x)*(positionA.y - positionC.y));

                float gamma = 1.0f - alpha - beta;

                if ((alpha > 0.0f) && (beta > 0.0f) & (gamma > 0.0f))
                {
                    collision = true;
                    break;
                }
            }

            if (collision)
            {
                int count = vertexData.vertexCount;
                Vector2 bodyPos = body.position;
                Vector2 *vertices = cast(Vector2*) PHYSAC_MALLOC(Vector2.sizeof * count);
                Mat2 trans = body.shape.transform;
                
                for (int i = 0; i < count; i++)
                    vertices[i] = vertexData.positions[i];

                // Destroy shattered physics body
                destroyPhysicsBody(body);

                for (int i = 0; i < count; i++)
                {
                    int nextIndex = (((i + 1) < count) ? (i + 1) : 0);
                    Vector2 center = triangleBarycenter(vertices[i], vertices[nextIndex], PHYSAC_VECTOR_ZERO);
                    center = vector2Add(bodyPos, center);
                    Vector2 offset = vector2Subtract(center, bodyPos);

                    PhysicsBody newBody = createPhysicsBodyPolygon(center, 10, 3, 10);     // Create polygon physics body with relevant values

                    PolygonData newData = { 0 };
                    newData.vertexCount = 3;

                    newData.positions[0] = vector2Subtract(vertices[i], offset);
                    newData.positions[1] = vector2Subtract(vertices[nextIndex], offset);
                    newData.positions[2] = vector2Subtract(position, center);

                    // Separate vertices to avoid unnecessary physics collisions
                    newData.positions[0].x *= 0.95f;
                    newData.positions[0].y *= 0.95f;
                    newData.positions[1].x *= 0.95f;
                    newData.positions[1].y *= 0.95f;
                    newData.positions[2].x *= 0.95f;
                    newData.positions[2].y *= 0.95f;

                    // Calculate polygon faces normals
                    for (int j = 0; j < newData.vertexCount; j++)
                    {
                        int nextVertex = (((j + 1) < newData.vertexCount) ? (j + 1) : 0);
                        Vector2 face = vector2Subtract(newData.positions[nextVertex], newData.positions[j]);

                        newData.normals[j] = Vector2(face.y, -face.x);
                        mathNormalize(&newData.normals[j]);
                    }

                    // Apply computed vertex data to new physics body shape
                    newBody.shape.vertexData = newData;
                    newBody.shape.transform = trans;

                    // Calculate centroid and moment of inertia
                    center = PHYSAC_VECTOR_ZERO;
                    float area = 0.0f;
                    float inertia = 0.0f;

                    for (int j = 0; j < newBody.shape.vertexData.vertexCount; j++)
                    {
                        // Triangle vertices, third vertex implied as (0, 0)
                        Vector2 p1 = newBody.shape.vertexData.positions[j];
                        int nextVertex = (((j + 1) < newBody.shape.vertexData.vertexCount) ? (j + 1) : 0);
                        Vector2 p2 = newBody.shape.vertexData.positions[nextVertex];

                        float D = mathCrossVector2(p1, p2);
                        float triangleArea = D/2;

                        area += triangleArea;

                        // Use area to weight the centroid average, not just vertex position
                        center.x += triangleArea*PHYSAC_K*(p1.x + p2.x);
                        center.y += triangleArea*PHYSAC_K*(p1.y + p2.y);

                        float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
                        float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
                        inertia += (0.25f*PHYSAC_K*D)*(intx2 + inty2);
                    }

                    center.x *= 1.0f/area;
                    center.y *= 1.0f/area;

                    newBody.mass = area;
                    newBody.inverseMass = ((newBody.mass != 0.0f) ? 1.0f/newBody.mass : 0.0f);
                    newBody.inertia = inertia;
                    newBody.inverseInertia = ((newBody.inertia != 0.0f) ? 1.0f/newBody.inertia : 0.0f);

                    // Calculate explosion force direction
                    Vector2 pointA = newBody.position;
                    Vector2 pointB = vector2Subtract(newData.positions[1], newData.positions[0]);
                    pointB.x /= 2.0f;
                    pointB.y /= 2.0f;
                    Vector2 forceDirection = vector2Subtract(vector2Add(pointA,
                                vector2Add(newData.positions[0], pointB)), newBody.position);
                    mathNormalize(&forceDirection);
                    forceDirection.x *= force;
                    forceDirection.y *= force;

                    // Apply force to new physics body
                    physicsAddForce(newBody, forceDirection);
                }

                PHYSAC_FREE(vertices);
            }
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] error when trying to shatter a null reference physics body");
        }
    }
}

// Returns the current amount of created physics bodies
int getPhysicsBodiesCount()
{
    return physicsBodiesCount;
}

// Returns a physics body of the bodies pool at a specific index
PhysicsBody getPhysicsBody(int index)
{
    if (index < physicsBodiesCount)
    {
        if (bodies[index] == null)
        {
            debug
            {
                printf("[PHYSAC] error when trying to get a null reference physics body");
            }
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] physics body index is out of bounds");
        }
    }

    return bodies[index];
}

// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
int getPhysicsShapeType(int index)
{
    int result = -1;

    if (index < physicsBodiesCount)
    {
        if (bodies[index] != null) 
        {
            result = bodies[index].shape.type;
        }
        else
        {
            debug
            {
                printf("[PHYSAC] error when trying to get a null reference physics body");
            }
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] physics body index is out of bounds");
        }
    }

    return result;
}

// Returns the amount of vertices of a physics body shape
int getPhysicsShapeVerticesCount(int index)
{
    int result = 0;

    if (index < physicsBodiesCount)
    {
        if (bodies[index] != null)
        {
            switch (bodies[index].shape.type)
            {
                case PhysicsShapeType.PHYSICS_CIRCLE: result = PHYSAC_CIRCLE_VERTICES; break;
                case PhysicsShapeType.PHYSICS_POLYGON: result = bodies[index].shape.vertexData.vertexCount; break;
                default: break;
            }
        }
        else
        {
            debug
            {
                printf("[PHYSAC] error when trying to get a null reference physics body");
            }
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] physics body index is out of bounds");
        }
    }

    return result;
}

// Returns transformed position of a body shape (body position + vertex transformed position)
Vector2 getPhysicsShapeVertex(PhysicsBody body, int vertex)
{
    Vector2 position = { 0.0f, 0.0f };

    if (body != null)
    {
        switch (body.shape.type)
        {
            case PhysicsShapeType.PHYSICS_CIRCLE:
            {
                position.x = body.position.x + cosf(360.0f/PHYSAC_CIRCLE_VERTICES*vertex*PHYSAC_DEG2RAD)*
                    body.shape.radius;
                position.y = body.position.y + sinf(360.0f/PHYSAC_CIRCLE_VERTICES*vertex*PHYSAC_DEG2RAD)*
                    body.shape.radius;
            } break;
            case PhysicsShapeType.PHYSICS_POLYGON:
            {
                PolygonData vertexData = body.shape.vertexData;
                position = vector2Add(body.position, mat2MultiplyVector2(body.shape.transform,
                            vertexData.positions[vertex]));
            } break;
            default: break;
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] error when trying to get a null reference physics body");
        }
    }

    return position;
}

// Sets physics body shape transform based on radians parameter
void setPhysicsBodyRotation(PhysicsBody body, float radians)
{
    if (body != null)
    {
        body.orient = radians;

        if (body.shape.type == PhysicsShapeType.PHYSICS_POLYGON)
            body.shape.transform = mat2Radians(radians);
    }
}

// Unitializes and destroys a physics body
void destroyPhysicsBody(PhysicsBody body)
{
    if (body != null)
    {
        int id = body.id;
        int index = -1;

        for (int i = 0; i < physicsBodiesCount; i++)
        {
            if (bodies[i].id == id)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            debug
            {
                printf("[PHYSAC] Not possible to find body id %i in pointers array\n", id);
            }
            return;
        }

        // Free body allocated memory
        PHYSAC_FREE(body);
        usedMemory -= PhysicsBodyData.sizeof;
        bodies[index] = null;

        // Reorder physics bodies pointers array and its catched index
        for (int i = index; i < physicsBodiesCount; i++)
        {
            if ((i + 1) < physicsBodiesCount)
                bodies[i] = bodies[i + 1];
        }

        // Update physics bodies count
        physicsBodiesCount--;

        debug
        {
            printf("[PHYSAC] destroyed physics body id %i\n", id);
        }
    }
    else
    {
        debug
        {
            printf("[PHYSAC] error trying to destroy a null referenced body\n");
        }
    }
}

// Unitializes physics pointers and exits physics loop thread
void closePhysics()
{
    // Exit physics loop thread
    physicsThreadEnabled = false;

    pthread_join(physicsThreadId, null);

    // Unitialize physics manifolds dynamic memory allocations
    for (int i = physicsManifoldsCount - 1; i >= 0; i--)
        destroyPhysicsManifold(contacts[i]);

    // Unitialize physics bodies dynamic memory allocations
    for (int i = physicsBodiesCount - 1; i >= 0; i--)
        destroyPhysicsBody(bodies[i]);

    debug
    {
        if (physicsBodiesCount > 0 || usedMemory != 0)
            printf("[PHYSAC] physics module closed with %i still allocated bodies [MEMORY: %i bytes]\n",
                    physicsBodiesCount, usedMemory);
        else if (physicsManifoldsCount > 0 || usedMemory != 0)
            printf("[PHYSAC] physics module closed with %i still allocated manifolds [MEMORY: %i bytes]\n",
                    physicsManifoldsCount, usedMemory);
        else
            printf("[PHYSAC] physics module closed successfully\n");
    }
}

//----------------------------------------------------------------------------------
// Module Internal Functions Definition
//----------------------------------------------------------------------------------
// Finds a valid index for a new physics body initialization
private int findAvailableBodyIndex()
{
    int index = -1;
    for (int i = 0; i < PHYSAC_MAX_BODIES; i++)
    {
        int currentId = i;

        // Check if current id already exist in other physics body
        for (int k = 0; k < physicsBodiesCount; k++)
        {
            if (bodies[k].id == currentId)
            {
                currentId++;
                break;
            }
        }

        // If it is not used, use it as new physics body id
        if (currentId == i)
        {
            index = i;
            break;
        }
    }

    return index;
}

// Creates a random polygon shape with max vertex distance from polygon pivot
private PolygonData createRandomPolygon(float radius, int sides)
{
    PolygonData data = { 0 };
    data.vertexCount = sides;

    // Calculate polygon vertices positions
    for (int i = 0; i < data.vertexCount; i++)
    {
        data.positions[i].x = cosf(360.0f/sides*i*PHYSAC_DEG2RAD)*radius;
        data.positions[i].y = sinf(360.0f/sides*i*PHYSAC_DEG2RAD)*radius;
    }

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < sides) ? (i + 1) : 0);
        Vector2 face = vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = Vector2(face.y, -face.x);
        mathNormalize(&data.normals[i]);
    }

    return data;
}

// Creates a rectangle polygon shape based on a min and max positions
private PolygonData createRectanglePolygon(Vector2 pos, Vector2 size)
{
    PolygonData data = { 0 };
    data.vertexCount = 4;

    // Calculate polygon vertices positions
    data.positions[0] = Vector2(pos.x + size.x/2, pos.y - size.y/2);
    data.positions[1] = Vector2(pos.x + size.x/2, pos.y + size.y/2);
    data.positions[2] = Vector2(pos.x - size.x/2, pos.y + size.y/2);
    data.positions[3] = Vector2(pos.x - size.x/2, pos.y - size.y/2);

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < data.vertexCount) ? (i + 1) : 0);
        Vector2 face = vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = Vector2(face.y, -face.x);
        mathNormalize(&data.normals[i]);
    }

    return data;
}

// Physics loop thread function
extern(C) private void* physicsLoop(void* arg)
/* private void physicsLoop() */
{
    debug
    {
        printf("[PHYSAC] physics thread created successfully\n");
    }

    // Initialize physics loop thread values
    physicsThreadEnabled = true;

    // Physics update loop
    while (physicsThreadEnabled)
    {
        runPhysicsStep();
    }

    return null;
}

// Physics steps calculations (dynamics, collisions and position corrections)
private void physicsStep()
{
    // Update current steps count
    stepsCount++;

    // Clear previous generated collisions information
    for (int i = physicsManifoldsCount - 1; i >= 0; i--)
    {
        PhysicsManifold manifold = contacts[i];
        
        if (manifold != null)
            destroyPhysicsManifold(manifold);
    }

    // Reset physics bodies grounded state
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody body = bodies[i];
        body.isGrounded = false;
    }

    // Generate new collision information
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody bodyA = bodies[i];

        if (bodyA != null)
        {
            for (int j = i + 1; j < physicsBodiesCount; j++)
            {
                PhysicsBody bodyB = bodies[j];

                if (bodyB != null)
                {
                    if ((bodyA.inverseMass == 0) && (bodyB.inverseMass == 0))
                        continue;

                    PhysicsManifold manifold = createPhysicsManifold(bodyA, bodyB);
                    solvePhysicsManifold(manifold);

                    if (manifold.contactsCount > 0)
                    {
                        // Create a new manifold with same information as previously solved manifold and add it to the manifolds pool last slot
                        PhysicsManifold newManifold = createPhysicsManifold(bodyA, bodyB);
                        newManifold.penetration = manifold.penetration;
                        newManifold.normal = manifold.normal;
                        newManifold.contacts[0] = manifold.contacts[0];
                        newManifold.contacts[1] = manifold.contacts[1];
                        newManifold.contactsCount = manifold.contactsCount;
                        newManifold.restitution = manifold.restitution;
                        newManifold.dynamicFriction = manifold.dynamicFriction;
                        newManifold.staticFriction = manifold.staticFriction;
                    }
                }
            }
        }
    }

    // Integrate forces to physics bodies
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody body = bodies[i];
        
        if (body != null)
            integratePhysicsForces(body);
    }

    // Initialize physics manifolds to solve collisions
    for (int i = 0; i < physicsManifoldsCount; i++)
    {
        PhysicsManifold manifold = contacts[i];
        
        if (manifold != null)
            initializePhysicsManifolds(manifold);
    }

    // Integrate physics collisions impulses to solve collisions
    for (int i = 0; i < PHYSAC_COLLISION_ITERATIONS; i++)
    {
        for (int j = 0; j < physicsManifoldsCount; j++)
        {
            PhysicsManifold manifold = contacts[i];
            
            if (manifold != null)
                integratePhysicsImpulses(manifold);
        }
    }

    // Integrate velocity to physics bodies
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody body = bodies[i];
        
        if (body != null)
            integratePhysicsVelocity(body);
    }

    // Correct physics bodies positions based on manifolds collision information
    for (int i = 0; i < physicsManifoldsCount; i++)
    {
        PhysicsManifold manifold = contacts[i];
        
        if (manifold != null)
            correctPhysicsPositions(manifold);
    }

    // Clear physics bodies forces
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody body = bodies[i];
        
        if (body != null)
        {
            body.force = PHYSAC_VECTOR_ZERO;
            body.torque = 0.0f;
        }
    }
}

// Wrapper to ensure PhysicsStep is run with at a fixed time step
void runPhysicsStep()
{
    // Calculate current time
    currentTime = getCurrentTime();

    // Calculate current delta time
    const double delta = currentTime - startTime;

    // Store the time elapsed since the last frame began
    accumulator += delta;

    // Fixed time stepping loop
    while (accumulator >= deltaTime)
    {
        physicsStep();
        accumulator -= deltaTime;
    }

    // Record the starting of this frame
    startTime = currentTime;
}

void setPhysicsTimeStep(double delta)
{
    deltaTime = delta;
}

// Finds a valid index for a new manifold initialization
private int findAvailableManifoldIndex()
{
    int index = -1;
    for (int i = 0; i < PHYSAC_MAX_MANIFOLDS; i++)
    {
        int currentId = i;

        // Check if current id already exist in other physics body
        for (int k = 0; k < physicsManifoldsCount; k++)
        {
            if (contacts[k].id == currentId)
            {
                currentId++;
                break;
            }
        }

        // If it is not used, use it as new physics body id
        if (currentId == i)
        {
            index = i;
            break;
        }
    }

    return index;
}

// Creates a new physics manifold to solve collision
private PhysicsManifold createPhysicsManifold(PhysicsBody a, PhysicsBody b)
{
    PhysicsManifold newManifold = cast(PhysicsManifold) PHYSAC_MALLOC(PhysicsManifoldData.sizeof);
    usedMemory += PhysicsManifoldData.sizeof;

    int newId = findAvailableManifoldIndex();
    if (newId != -1)
    {
        // Initialize new manifold with generic values
        newManifold.id = newId;
        newManifold.bodyA = a;
        newManifold.bodyB = b;
        newManifold.penetration = 0;
        newManifold.normal = PHYSAC_VECTOR_ZERO;
        newManifold.contacts[0] = PHYSAC_VECTOR_ZERO;
        newManifold.contacts[1] = PHYSAC_VECTOR_ZERO;
        newManifold.contactsCount = 0;
        newManifold.restitution = 0.0f;
        newManifold.dynamicFriction = 0.0f;
        newManifold.staticFriction = 0.0f;

        // Add new body to bodies pointers array and update bodies count
        contacts[physicsManifoldsCount] = newManifold;
        physicsManifoldsCount++;
    }
    else
    {
        debug
        {
            printf("[PHYSAC] new physics manifold creation failed because there is any available id to use\n");
        }
    }

    return newManifold;
}

// Unitializes and destroys a physics manifold
private void destroyPhysicsManifold(PhysicsManifold manifold)
{
    if (manifold != null)
    {
        int id = manifold.id;
        int index = -1;

        for (int i = 0; i < physicsManifoldsCount; i++)
        {
            if (contacts[i].id == id)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            debug
            {
                printf("[PHYSAC] Not possible to manifold id %i in pointers array\n", id);
            }
            return;
        }      

        // Free manifold allocated memory
        PHYSAC_FREE(manifold);
        usedMemory -= PhysicsManifoldData.sizeof;
        contacts[index] = null;

        // Reorder physics manifolds pointers array and its catched index
        for (int i = index; i < physicsManifoldsCount; i++)
        {
            if ((i + 1) < physicsManifoldsCount)
                contacts[i] = contacts[i + 1];
        }

        // Update physics manifolds count
        physicsManifoldsCount--;
    }
    else
    {
        debug
        {
            printf("[PHYSAC] error trying to destroy a null referenced manifold\n");
        }
    }
}

// Solves a created physics manifold between two physics bodies
private void solvePhysicsManifold(PhysicsManifold manifold)
{
    switch (manifold.bodyA.shape.type)
    {
        case PhysicsShapeType.PHYSICS_CIRCLE:
        {
            switch (manifold.bodyB.shape.type)
            {
                case PhysicsShapeType.PHYSICS_CIRCLE: solveCircleToCircle(manifold); break;
                case PhysicsShapeType.PHYSICS_POLYGON: solveCircleToPolygon(manifold); break;
                default: break;
            }
        } break;
        case PhysicsShapeType.PHYSICS_POLYGON:
        {
            switch (manifold.bodyB.shape.type)
            {
                case PhysicsShapeType.PHYSICS_CIRCLE: solvePolygonToCircle(manifold); break;
                case PhysicsShapeType.PHYSICS_POLYGON: solvePolygonToPolygon(manifold); break;
                default: break;
            }
        } break;
        default: break;
    }

    // Update physics body grounded state if normal direction is down and grounded state is not set yet in previous manifolds
    if (!manifold.bodyB.isGrounded)
        manifold.bodyB.isGrounded = (manifold.normal.y < 0);
}

// Solves collision between two circle shape physics bodies
private void solveCircleToCircle(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    // Calculate translational vector, which is normal
    Vector2 normal = vector2Subtract(bodyB.position, bodyA.position);

    float distSqr = mathLenSqr(normal);
    float radius = bodyA.shape.radius + bodyB.shape.radius;

    // Check if circles are not in contact
    if (distSqr >= radius*radius)
    {
        manifold.contactsCount = 0;
        return;
    }

    float distance = sqrtf(distSqr);
    manifold.contactsCount = 1;

    if (distance == 0.0f)
    {
        manifold.penetration = bodyA.shape.radius;
        manifold.normal = Vector2(1.0f, 0.0f);
        manifold.contacts[0] = bodyA.position;
    }
    else
    {
        manifold.penetration = radius - distance;
        manifold.normal = Vector2(normal.x/distance, normal.y/distance); // Faster than using MathNormalize() due to sqrt is already performed
        manifold.contacts[0] = Vector2(manifold.normal.x*bodyA.shape.radius + bodyA.position.x,
                manifold.normal.y*bodyA.shape.radius + bodyA.position.y);
    }

    // Update physics body grounded state if normal direction is down
    if (!bodyA.isGrounded)
        bodyA.isGrounded = (manifold.normal.y < 0);
}

// Solves collision between a circle to a polygon shape physics bodies
private void solveCircleToPolygon(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    solveDifferentShapes(manifold, bodyA, bodyB);
}

// Solves collision between a circle to a polygon shape physics bodies
private void solvePolygonToCircle(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    solveDifferentShapes(manifold, bodyB, bodyA);
    
    manifold.normal.x *= -1.0f;
    manifold.normal.y *= -1.0f;
}

// Solve collision between two different types of shapes
private void solveDifferentShapes(PhysicsManifold manifold, PhysicsBody bodyA, PhysicsBody bodyB)
{
    manifold.contactsCount = 0;

    // Transform circle center to polygon transform space
    Vector2 center = bodyA.position;
    center = mat2MultiplyVector2(mat2Transpose(bodyB.shape.transform), vector2Subtract(center, bodyB.position));

    // Find edge with minimum penetration
    // It is the same concept as using support points in SolvePolygonToPolygon
    float separation = -PHYSAC_FLT_MAX;
    int faceNormal = 0;
    PolygonData vertexData = bodyB.shape.vertexData;

    for (int i = 0; i < vertexData.vertexCount; i++)
    {
        float currentSeparation = mathDot(vertexData.normals[i], vector2Subtract(center, vertexData.positions[i]));

        if (currentSeparation > bodyA.shape.radius)
            return;

        if (currentSeparation > separation)
        {
            separation = currentSeparation;
            faceNormal = i;
        }
    }

    // Grab face's vertices
    Vector2 v1 = vertexData.positions[faceNormal];
    int nextIndex = (((faceNormal + 1) < vertexData.vertexCount) ? (faceNormal + 1) : 0);
    Vector2 v2 = vertexData.positions[nextIndex];

    // Check to see if center is within polygon
    if (separation < PHYSAC_EPSILON)
    {
        manifold.contactsCount = 1;
        Vector2 normal = mat2MultiplyVector2(bodyB.shape.transform, vertexData.normals[faceNormal]);
        manifold.normal = Vector2(-normal.x, -normal.y);
        manifold.contacts[0] = Vector2(manifold.normal.x*bodyA.shape.radius + bodyA.position.x,
                manifold.normal.y*bodyA.shape.radius + bodyA.position.y);
        manifold.penetration = bodyA.shape.radius;
        return;
    }

    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = mathDot(vector2Subtract(center, v1), vector2Subtract(v2, v1));
    float dot2 = mathDot(vector2Subtract(center, v2), vector2Subtract(v1, v2));
    manifold.penetration = bodyA.shape.radius - separation;

    if (dot1 <= 0.0f) // Closest to v1
    {
        if (distSqr(center, v1) > bodyA.shape.radius*bodyA.shape.radius)
            return;

        manifold.contactsCount = 1;
        Vector2 normal = vector2Subtract(v1, center);
        normal = mat2MultiplyVector2(bodyB.shape.transform, normal);
        mathNormalize(&normal);
        manifold.normal = normal;
        v1 = mat2MultiplyVector2(bodyB.shape.transform, v1);
        v1 = vector2Add(v1, bodyB.position);
        manifold.contacts[0] = v1;
    }
    else if (dot2 <= 0.0f) // Closest to v2
    {
        if (distSqr(center, v2) > bodyA.shape.radius*bodyA.shape.radius)
            return;

        manifold.contactsCount = 1;
        Vector2 normal = vector2Subtract(v2, center);
        v2 = mat2MultiplyVector2(bodyB.shape.transform, v2);
        v2 = vector2Add(v2, bodyB.position);
        manifold.contacts[0] = v2;
        normal = mat2MultiplyVector2(bodyB.shape.transform, normal);
        mathNormalize(&normal);
        manifold.normal = normal;
    }
    else // Closest to face
    {
        Vector2 normal = vertexData.normals[faceNormal];

        if (mathDot(vector2Subtract(center, v1), normal) > bodyA.shape.radius)
            return;

        normal = mat2MultiplyVector2(bodyB.shape.transform, normal);
        manifold.normal = Vector2(-normal.x, -normal.y);
        manifold.contacts[0] = Vector2(manifold.normal.x*bodyA.shape.radius + bodyA.position.x,
                manifold.normal.y*bodyA.shape.radius + bodyA.position.y);
        manifold.contactsCount = 1;
    }
}

// Solves collision between two polygons shape physics bodies
private void solvePolygonToPolygon(PhysicsManifold manifold)
{
    if ((manifold.bodyA == null) || (manifold.bodyB == null))
        return;

    PhysicsShape bodyA = manifold.bodyA.shape;
    PhysicsShape bodyB = manifold.bodyB.shape;
    manifold.contactsCount = 0;

    // Check for separating axis with A shape's face planes
    int faceA = 0;
    float penetrationA = findAxisLeastPenetration(&faceA, bodyA, bodyB);
    
    if (penetrationA >= 0.0f)
        return;

    // Check for separating axis with B shape's face planes
    int faceB = 0;
    float penetrationB = findAxisLeastPenetration(&faceB, bodyB, bodyA);
    
    if (penetrationB >= 0.0f)
        return;

    int referenceIndex = 0;
    bool flip = false;  // Always point from A shape to B shape

    PhysicsShape refPoly; // Reference
    PhysicsShape incPoly; // Incident

    // Determine which shape contains reference face
    if (biasGreaterThan(penetrationA, penetrationB))
    {
        refPoly = bodyA;
        incPoly = bodyB;
        referenceIndex = faceA;
    }
    else
    {
        refPoly = bodyB;
        incPoly = bodyA;
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    Vector2[2] incidentFace;
    findIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex);

    // Setup reference face vertices
    PolygonData refData = refPoly.vertexData;
    Vector2 v1 = refData.positions[referenceIndex];
    referenceIndex = (((referenceIndex + 1) < refData.vertexCount) ? (referenceIndex + 1) : 0);
    Vector2 v2 = refData.positions[referenceIndex];

    // Transform vertices to world space
    v1 = mat2MultiplyVector2(refPoly.transform, v1);
    v1 = vector2Add(v1, refPoly.body.position);
    v2 = mat2MultiplyVector2(refPoly.transform, v2);
    v2 = vector2Add(v2, refPoly.body.position);

    // Calculate reference face side normal in world space
    Vector2 sidePlaneNormal = vector2Subtract(v2, v1);
    mathNormalize(&sidePlaneNormal);

    // Orthogonalize
    Vector2 refFaceNormal = { sidePlaneNormal.y, -sidePlaneNormal.x };
    float refC = mathDot(refFaceNormal, v1);
    float negSide = mathDot(sidePlaneNormal, v1)*-1;
    float posSide = mathDot(sidePlaneNormal, v2);

    // Clip incident face to reference face side planes (due to floating point error, possible to not have required points
    if (clip(Vector2(-sidePlaneNormal.x, -sidePlaneNormal.y), negSide, &incidentFace[0], &incidentFace[1]) < 2)
        return;

    if (clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2)
        return;

    // Flip normal if required
    manifold.normal = (flip ? Vector2(-refFaceNormal.x, -refFaceNormal.y) : refFaceNormal);

    // Keep points behind reference face
    int currentPoint = 0; // Clipped points behind reference face
    float separation = mathDot(refFaceNormal, incidentFace[0]) - refC;
    
    if (separation <= 0.0f)
    {
        manifold.contacts[currentPoint] = incidentFace[0];
        manifold.penetration = -separation;
        currentPoint++;
    }
    else
        manifold.penetration = 0.0f;

    separation = mathDot(refFaceNormal, incidentFace[1]) - refC;

    if (separation <= 0.0f)
    {
        manifold.contacts[currentPoint] = incidentFace[1];
        manifold.penetration += -separation;
        currentPoint++;

        // Calculate total penetration average
        manifold.penetration /= currentPoint;
    }

    manifold.contactsCount = currentPoint;
}

// Integrates physics forces into velocity
private void integratePhysicsForces(PhysicsBody body)
{
    if ((body == null) || (body.inverseMass == 0.0f) || !body.enabled)
        return;

    body.velocity.x += (body.force.x*body.inverseMass)*(deltaTime/2.0);
    body.velocity.y += (body.force.y*body.inverseMass)*(deltaTime/2.0);

    if (body.useGravity)
    {
        body.velocity.x += gravityForce.x*(deltaTime/1000/2.0);
        body.velocity.y += gravityForce.y*(deltaTime/1000/2.0);
    }

    if (!body.freezeOrient)
        body.angularVelocity += body.torque*body.inverseInertia*(deltaTime/2.0);
}

// Initializes physics manifolds to solve collisions
private void initializePhysicsManifolds(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    // Calculate average restitution, static and dynamic friction
    manifold.restitution = sqrtf(bodyA.restitution*bodyB.restitution);
    manifold.staticFriction = sqrtf(bodyA.staticFriction*bodyB.staticFriction);
    manifold.dynamicFriction = sqrtf(bodyA.dynamicFriction*bodyB.dynamicFriction);

    for (int i = 0; i < manifold.contactsCount; i++)
    {
        // Caculate radius from center of mass to contact
        Vector2 radiusA = vector2Subtract(manifold.contacts[i], bodyA.position);
        Vector2 radiusB = vector2Subtract(manifold.contacts[i], bodyB.position);

        Vector2 crossA = mathCross(bodyA.angularVelocity, radiusA);
        Vector2 crossB = mathCross(bodyB.angularVelocity, radiusB);

        Vector2 radiusV = { 0.0f, 0.0f };
        radiusV.x = bodyB.velocity.x + crossB.x - bodyA.velocity.x - crossA.x;
        radiusV.y = bodyB.velocity.y + crossB.y - bodyA.velocity.y - crossA.y;

        // Determine if we should perform a resting collision or not;
        // The idea is if the only thing moving this object is gravity, then the collision should be performed without any restitution
        if (mathLenSqr(radiusV) < (mathLenSqr(Vector2(gravityForce.x*deltaTime/1000,
                            gravityForce.y*deltaTime/1000)) + PHYSAC_EPSILON))
            manifold.restitution = 0;
    }
}

// Integrates physics collisions impulses to solve collisions
private void integratePhysicsImpulses(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    // Early out and positional correct if both objects have infinite mass
    if (fabs(bodyA.inverseMass + bodyB.inverseMass) <= PHYSAC_EPSILON)
    {
        bodyA.velocity = PHYSAC_VECTOR_ZERO;
        bodyB.velocity = PHYSAC_VECTOR_ZERO;
        return;
    }

    for (int i = 0; i < manifold.contactsCount; i++)
    {
        // Calculate radius from center of mass to contact
        Vector2 radiusA = vector2Subtract(manifold.contacts[i], bodyA.position);
        Vector2 radiusB = vector2Subtract(manifold.contacts[i], bodyB.position);

        // Calculate relative velocity
        Vector2 radiusV = { 0.0f, 0.0f };
        radiusV.x = bodyB.velocity.x + mathCross(bodyB.angularVelocity, radiusB).x - bodyA.velocity.x -
            mathCross(bodyA.angularVelocity, radiusA).x;
        radiusV.y = bodyB.velocity.y + mathCross(bodyB.angularVelocity, radiusB).y - bodyA.velocity.y -
            mathCross(bodyA.angularVelocity, radiusA).y;

        // Relative velocity along the normal
        float contactVelocity = mathDot(radiusV, manifold.normal);

        // Do not resolve if velocities are separating
        if (contactVelocity > 0.0f)
            return;

        float raCrossN = mathCrossVector2(radiusA, manifold.normal);
        float rbCrossN = mathCrossVector2(radiusB, manifold.normal);

        float inverseMassSum = bodyA.inverseMass + bodyB.inverseMass + (raCrossN*raCrossN)*bodyA.inverseInertia +
            (rbCrossN*rbCrossN)*bodyB.inverseInertia;

        // Calculate impulse scalar value
        float impulse = -(1.0f + manifold.restitution)*contactVelocity;
        impulse /= inverseMassSum;
        impulse /= cast(float) manifold.contactsCount;

        // Apply impulse to each physics body
        Vector2 impulseV = { manifold.normal.x*impulse, manifold.normal.y*impulse };

        if (bodyA.enabled)
        {
            bodyA.velocity.x += bodyA.inverseMass*(-impulseV.x);
            bodyA.velocity.y += bodyA.inverseMass*(-impulseV.y);
            
            if (!bodyA.freezeOrient)
                bodyA.angularVelocity += bodyA.inverseInertia*mathCrossVector2(radiusA,
                        Vector2(-impulseV.x, -impulseV.y));
        }

        if (bodyB.enabled)
        {
            bodyB.velocity.x += bodyB.inverseMass*(impulseV.x);
            bodyB.velocity.y += bodyB.inverseMass*(impulseV.y);
            
            if (!bodyB.freezeOrient)
                bodyB.angularVelocity += bodyB.inverseInertia*mathCrossVector2(radiusB, impulseV);
        }

        // Apply friction impulse to each physics body
        radiusV.x = bodyB.velocity.x + mathCross(bodyB.angularVelocity, radiusB).x - bodyA.velocity.x -
            mathCross(bodyA.angularVelocity, radiusA).x;
        radiusV.y = bodyB.velocity.y + mathCross(bodyB.angularVelocity, radiusB).y - bodyA.velocity.y -
            mathCross(bodyA.angularVelocity, radiusA).y;

        Vector2 tangent = { radiusV.x - (manifold.normal.x*mathDot(radiusV, manifold.normal)), radiusV.y -
            (manifold.normal.y*mathDot(radiusV, manifold.normal)) };
        mathNormalize(&tangent);

        // Calculate impulse tangent magnitude
        float impulseTangent = -mathDot(radiusV, tangent);
        impulseTangent /= inverseMassSum;
        impulseTangent /= cast(float) manifold.contactsCount;

        float absImpulseTangent = fabs(impulseTangent);

        // Don't apply tiny friction impulses
        if (absImpulseTangent <= PHYSAC_EPSILON)
            return;

        // Apply coulumb's law
        Vector2 tangentImpulse = { 0.0f, 0.0f };
        if (absImpulseTangent < impulse*manifold.staticFriction)
            tangentImpulse = Vector2(tangent.x*impulseTangent, tangent.y*impulseTangent);
        else
            tangentImpulse = Vector2(tangent.x*-impulse*manifold.dynamicFriction,
                    tangent.y*-impulse*manifold.dynamicFriction);

        // Apply friction impulse
        if (bodyA.enabled)
        {
            bodyA.velocity.x += bodyA.inverseMass*(-tangentImpulse.x);
            bodyA.velocity.y += bodyA.inverseMass*(-tangentImpulse.y);

            if (!bodyA.freezeOrient)
                bodyA.angularVelocity += bodyA.inverseInertia*mathCrossVector2(radiusA,
                        Vector2(-tangentImpulse.x, -tangentImpulse.y));
        }

        if (bodyB.enabled)
        {
            bodyB.velocity.x += bodyB.inverseMass*(tangentImpulse.x);
            bodyB.velocity.y += bodyB.inverseMass*(tangentImpulse.y);

            if (!bodyB.freezeOrient)
                bodyB.angularVelocity += bodyB.inverseInertia*mathCrossVector2(radiusB, tangentImpulse);
        }
    }
}

// Integrates physics velocity into position and forces
private void integratePhysicsVelocity(PhysicsBody body)
{
    if ((body == null) ||!body.enabled)
        return;

    body.position.x += body.velocity.x*deltaTime;
    body.position.y += body.velocity.y*deltaTime;

    if (!body.freezeOrient)
        body.orient += body.angularVelocity*deltaTime;

    mat2Set(&body.shape.transform, body.orient);

    integratePhysicsForces(body);
}

// Corrects physics bodies positions based on manifolds collision information
private void correctPhysicsPositions(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold.bodyA;
    PhysicsBody bodyB = manifold.bodyB;

    if ((bodyA == null) || (bodyB == null))
        return;

    Vector2 correction = { 0.0f, 0.0f };
    correction.x = (max(manifold.penetration - PHYSAC_PENETRATION_ALLOWANCE, 0.0f)/
            (bodyA.inverseMass + bodyB.inverseMass))*manifold.normal.x*PHYSAC_PENETRATION_CORRECTION;
    correction.y = (max(manifold.penetration - PHYSAC_PENETRATION_ALLOWANCE, 0.0f)/
            (bodyA.inverseMass + bodyB.inverseMass))*manifold.normal.y*PHYSAC_PENETRATION_CORRECTION;

    if (bodyA.enabled)
    {
        bodyA.position.x -= correction.x*bodyA.inverseMass;
        bodyA.position.y -= correction.y*bodyA.inverseMass;
    }

    if (bodyB.enabled)
    {
        bodyB.position.x += correction.x*bodyB.inverseMass;
        bodyB.position.y += correction.y*bodyB.inverseMass;
    }
}

// Returns the extreme point along a direction within a polygon
private Vector2 getSupport(PhysicsShape shape, Vector2 dir)
{
    float bestProjection = -PHYSAC_FLT_MAX;
    Vector2 bestVertex = { 0.0f, 0.0f };
    PolygonData data = shape.vertexData;

    for (int i = 0; i < data.vertexCount; i++)
    {
        Vector2 vertex = data.positions[i];
        float projection = mathDot(vertex, dir);

        if (projection > bestProjection)
        {
            bestVertex = vertex;
            bestProjection = projection;
        }
    }

    return bestVertex;
}

// Finds polygon shapes axis least penetration
float findAxisLeastPenetration(int* faceIndex, PhysicsShape shapeA, PhysicsShape shapeB)
{
    float bestDistance = -PHYSAC_FLT_MAX;
    int bestIndex = 0;

    PolygonData dataA = shapeA.vertexData;

    for (int i = 0; i < dataA.vertexCount; i++)
    {
        // Retrieve a face normal from A shape
        Vector2 normal = dataA.normals[i];
        Vector2 transNormal = mat2MultiplyVector2(shapeA.transform, normal);

        // Transform face normal into B shape's model space
        Mat2 buT = mat2Transpose(shapeB.transform);
        normal = mat2MultiplyVector2(buT, transNormal);

        // Retrieve support point from B shape along -n
        Vector2 support = getSupport(shapeB, Vector2(-normal.x, -normal.y));

        // Retrieve vertex on face from A shape, transform into B shape's model space
        Vector2 vertex = dataA.positions[i];
        vertex = mat2MultiplyVector2(shapeA.transform, vertex);
        vertex = vector2Add(vertex, shapeA.body.position);
        vertex = vector2Subtract(vertex, shapeB.body.position);
        vertex = mat2MultiplyVector2(buT, vertex);

        // Compute penetration distance in B shape's model space
        float distance = mathDot(normal, vector2Subtract(support, vertex));

        // Store greatest distance
        if (distance > bestDistance)
        {
            bestDistance = distance;
            bestIndex = i;
        }
    }

    *faceIndex = bestIndex;
    return bestDistance;
}

// Finds two polygon shapes incident face
void findIncidentFace(Vector2* v0, Vector2* v1, PhysicsShape reff, PhysicsShape inc, int index)
{
    PolygonData refData = reff.vertexData;
    PolygonData incData = inc.vertexData;

    Vector2 referenceNormal = refData.normals[index];

    // Calculate normal in incident's frame of reference
    referenceNormal = mat2MultiplyVector2(reff.transform, referenceNormal); // To world space
    referenceNormal = mat2MultiplyVector2(mat2Transpose(inc.transform), referenceNormal); // To incident's model space

    // Find most anti-normal face on polygon
    int incidentFace = 0;
    float minDot = PHYSAC_FLT_MAX;

    for (int i = 0; i < incData.vertexCount; i++)
    {
        float dot = mathDot(referenceNormal, incData.normals[i]);

        if (dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    // Assign face vertices for incident face
    *v0 = mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v0 = vector2Add(*v0, inc.body.position);
    incidentFace = (((incidentFace + 1) < incData.vertexCount) ? (incidentFace + 1) : 0);
    *v1 = mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v1 = vector2Add(*v1, inc.body.position);
}

// Calculates clipping based on a normal and two faces
private int clip(Vector2 normal, float clip, Vector2* faceA, Vector2* faceB)
{
    int sp = 0;
    Vector2[2] outt = [*faceA, *faceB];

    // Retrieve distances from each endpoint to the line
    float distanceA = mathDot(normal, *faceA) - clip;
    float distanceB = mathDot(normal, *faceB) - clip;

    // If negative (behind plane)
    if (distanceA <= 0.0f)
        outt[sp++] = *faceA;

    if (distanceB <= 0.0f)
        outt[sp++] = *faceB;

    // If the points are on different sides of the plane
    if ((distanceA*distanceB) < 0.0f)
    {
        // Push intersection point
        float alpha = distanceA/(distanceA - distanceB);
        outt[sp] = *faceA;
        Vector2 delta = vector2Subtract(*faceB, *faceA);
        delta.x *= alpha;
        delta.y *= alpha;
        outt[sp] = vector2Add(outt[sp], delta);
        sp++;
    }

    // Assign the new converted values
    *faceA = outt[0];
    *faceB = outt[1];

    return sp;
}

// Check if values are between bias range
private bool biasGreaterThan(float valueA, float valueB)
{
    return (valueA >= (valueB*0.95f + valueA*0.01f));
}

// Returns the barycenter of a triangle given by 3 points
private Vector2 triangleBarycenter(Vector2 v1, Vector2 v2, Vector2 v3)
{
    Vector2 result = { 0.0f, 0.0f };

    result.x = (v1.x + v2.x + v3.x)/3;
    result.y = (v1.y + v2.y + v3.y)/3;

    return result;
}

// Initializes hi-resolution MONOTONIC timer
private void initTimer()
{
    srand(cast(uint) time(null)); // Initialize random seed

    frequency = MonoTime.ticksPerSecond();

    baseTime = getTimeCount();      // Get MONOTONIC clock time offset
    startTime = getCurrentTime();   // Get current time
}

// Get hi-res MONOTONIC time measure in seconds
private ulong getTimeCount()
{
    return MonoTime.currTime().ticks();
}

// Get current time in milliseconds
private double getCurrentTime()
{
    return cast(double)(getTimeCount() - baseTime)/frequency*1000;
}

// Returns the cross product of a vector and a value
private Vector2 mathCross(float value, Vector2 vector)
{
    return Vector2(-value*vector.y, value*vector.x);
}

// Returns the cross product of two vectors
private float mathCrossVector2(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.y - v1.y*v2.x);
}

// Returns the len square root of a vector
private float mathLenSqr(Vector2 vector)
{
    return (vector.x*vector.x + vector.y*vector.y);
}

// Returns the dot product of two vectors
private float mathDot(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.x + v1.y*v2.y);
}

// Returns the square root of distance between two vectors
private float distSqr(Vector2 v1, Vector2 v2)
{
    Vector2 dir = vector2Subtract(v1, v2);
    return mathDot(dir, dir);
}

// Returns the normalized values of a vector
private void mathNormalize(Vector2* vector)
{
    float length, ilength;

    Vector2 aux = *vector;
    length = sqrtf(aux.x*aux.x + aux.y*aux.y);

    if (length == 0)
        length = 1.0f;

    ilength = 1.0f/length;

    vector.x *= ilength;
    vector.y *= ilength;
}

// Returns the sum of two given vectors
private Vector2 vector2Add(Vector2 v1, Vector2 v2)
{
    return Vector2(v1.x + v2.x, v1.y + v2.y);
}

// Returns the subtract of two given vectors
private Vector2 vector2Subtract(Vector2 v1, Vector2 v2)
{
    return Vector2(v1.x - v2.x, v1.y - v2.y);
}

// Creates a matrix 2x2 from a given radians value
private Mat2 mat2Radians(float radians)
{
    float c = cosf(radians);
    float s = sinf(radians);

    return Mat2(c, -s, s, c);
}

// Set values from radians to a created matrix 2x2
private void mat2Set(Mat2* matrix, float radians)
{
    float cos = cosf(radians);
    float sin = sinf(radians);

    matrix.m00 = cos;
    matrix.m01 = -sin;
    matrix.m10 = sin;
    matrix.m11 = cos;
}

// Returns the transpose of a given matrix 2x2
private Mat2 mat2Transpose(Mat2 matrix)
{
    return Mat2(matrix.m00, matrix.m10, matrix.m01, matrix.m11);
}

// Multiplies a vector by a matrix 2x2
private Vector2 mat2MultiplyVector2(Mat2 matrix, Vector2 vector)
{
    return Vector2(matrix.m00*vector.x + matrix.m01*vector.y, matrix.m10*vector.x + matrix.m11*vector.y);
}
