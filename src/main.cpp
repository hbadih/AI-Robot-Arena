
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <box2d/box2d.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>


constexpr int CELL_SIZE = 50;
constexpr int GRID_COLS = 16;
constexpr int GRID_ROWS = 12;


struct Obstacle {
    float x;
    float y;
    float width;
    float height;
};

// Shared obstacle collection
std::vector<Obstacle> obstacles = {
    {500.0f, 225.0f, 100.0f, 150.0f},
    {200.0f, 100.0f, 100.0f, 100.0f},
    {250.0f, 400.0f, 100.0f, 100.0f}
};


struct GridCell {
    int row;
    int col;
};

struct AStarNode {
    int row;
    int col;

    float gCost; // Distance traveled from start
    float hCost; // Estimated distance to goal

    float fCost() const {
        return gCost + hCost;
    }
};


std::vector<GridCell> findPath(
    GridCell start,
    GridCell goal,
    GridCell avoidCell = {-1, -1}
) {
    const float INF = std::numeric_limits<float>::infinity();

    float gScore[GRID_ROWS][GRID_COLS];
    bool closed[GRID_ROWS][GRID_COLS] = {};
    bool blocked[GRID_ROWS][GRID_COLS] = {};
    GridCell parent[GRID_ROWS][GRID_COLS] = {};

    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c < GRID_COLS; c++) {
            gScore[r][c] = INF;

            // Mark grid cells that are blocked by obstacles.
            // Include clearance for the robot's 50x50 size
            float centerX = c * CELL_SIZE + CELL_SIZE / 2.0f;
            float centerY = r * CELL_SIZE + CELL_SIZE / 2.0f;
            
            blocked[r][c] = false;

            // Check every obstacle in the shared collection
            for (const Obstacle& obstacle : obstacles) {

                // Expand obstacle boundaries by half the robot size
                const float clearance = 25.0f;

                float left = obstacle.x - clearance;
                float right = obstacle.x + obstacle.width + clearance;

                float top = obstacle.y - clearance;
                float bottom = obstacle.y + obstacle.height + clearance;

                if (centerX >= left && centerX <= right &&
                    centerY >= top && centerY <= bottom) {

                    blocked[r][c] = true;
                    break;
                }
            }
        }
    }

    auto valid = [&](int r, int c) {
        return r >= 0 && r < GRID_ROWS &&
            c >= 0 && c < GRID_COLS &&
            !(r == avoidCell.row && c == avoidCell.col) &&
            !blocked[r][c];
    };

    if (!valid(start.row, start.col) ||
        !valid(goal.row, goal.col)) {
        return {};
    }

    auto heuristic = [&](int r, int c) {
        return static_cast<float>(
            std::abs(r - goal.row) +
            std::abs(c - goal.col)
        );
    };

    auto compare = [](const AStarNode& a, const AStarNode& b) {
        return a.fCost() > b.fCost();
    };

    std::priority_queue<
        AStarNode,
        std::vector<AStarNode>,
        decltype(compare)
    > open(compare);

    gScore[start.row][start.col] = 0.0f;

    open.push({
        start.row, start.col,
        0.0f,
        heuristic(start.row, start.col)
    });

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int r = current.row;
        int c = current.col;

        if (closed[r][c]) {
            continue;
        }

        closed[r][c] = true;

        if (r == goal.row && c == goal.col) {
            std::vector<GridCell> path;

            GridCell cell = goal;

            while (cell.row != start.row ||
                   cell.col != start.col) {
                path.push_back(cell);
                cell = parent[cell.row][cell.col];
            }

            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 4; i++) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (!valid(nr, nc) || closed[nr][nc]) {
                continue;
            }

            float newCost = gScore[r][c] + 1.0f;

            if (newCost < gScore[nr][nc]) {
                gScore[nr][nc] = newCost;
                parent[nr][nc] = {r, c};

                open.push({
                    nr, nc,
                    newCost,
                    heuristic(nr, nc)
                });
            }
        }
    }

    return {}; // No path found
}

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL initialization failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "AI Robot Arena", 800, 600, 0
    );

    if (!window) {
        SDL_Log("Window creation failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    if (!renderer) {
        SDL_Log("Renderer creation failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Robot properties
    float robotX = 375.0f;
    float robotY = 275.0f;
    const float robotSize = 50.0f;
    
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = {0.0f, 0.0f};
    b2WorldId worldId = b2CreateWorld(&worldDef);
    

    // Create a dynamic robot body
    b2BodyDef robotDef = b2DefaultBodyDef();
    robotDef.type = b2_dynamicBody;

    // Box2D uses meters, not pixels.
    // 100 pixels = 1 meter.
    robotDef.position = {4.0f, 3.0f};
    robotDef.fixedRotation = true;

    b2BodyId robotBody = b2CreateBody(worldId, &robotDef);

    // Create a square collision shape
    b2Polygon robotBox = b2MakeBox(0.25f, 0.25f);

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.material.friction = 0.3f;

    b2CreatePolygonShape(robotBody, &shapeDef, &robotBox);

 
    // Create physical walls around the arena
    auto createWall = [&](float x, float y,
                        float halfWidth, float halfHeight) {
        b2BodyDef wallDef = b2DefaultBodyDef();
        wallDef.position = {x, y};

        // Static bodies do not move
        b2BodyId wallBody = b2CreateBody(worldId, &wallDef);

        b2Polygon wallBox = b2MakeBox(halfWidth, halfHeight);
        b2ShapeDef wallShape = b2DefaultShapeDef();

        b2CreatePolygonShape(wallBody, &wallShape, &wallBox);
    };

    // Arena dimensions: 8 meters x 6 meters
    // Conversion: 100 pixels = 1 meter

    // Top wall
    createWall(4.0f, -0.1f, 4.0f, 0.1f);

    // Bottom wall
    createWall(4.0f, 6.1f, 4.0f, 0.1f);

    // Left wall
    createWall(-0.1f, 3.0f, 0.1f, 3.0f);

    // Right wall
    createWall(8.1f, 3.0f, 0.1f, 3.0f);

    
    
    // Create Box2D bodies for all obstacles
    for (const Obstacle& obstacle : obstacles) {

        b2BodyDef obstacleDef = b2DefaultBodyDef();

        // Calculate the obstacle's center in pixels
        float centerX = obstacle.x + obstacle.width / 2.0f;
        float centerY = obstacle.y + obstacle.height / 2.0f;

        // Convert pixels to Box2D meters
        obstacleDef.position = {
            centerX / 100.0f,
            centerY / 100.0f
        };

        // Create a static Box2D body
        b2BodyId obstacleBody =
            b2CreateBody(worldId, &obstacleDef);

        // Create the rectangular collision shape
        b2Polygon obstacleBox = b2MakeBox(
            obstacle.width / 200.0f,
            obstacle.height / 200.0f
        );

        b2ShapeDef obstacleShapeDef = b2DefaultShapeDef();

        b2CreatePolygonShape(
            obstacleBody,
            &obstacleShapeDef,
            &obstacleBox
        );
    }

   
    SDL_Log("Box2D physics world created successfully!");
    // Calculate a path from the robot to the green goal
    GridCell start = {6, 8};
    GridCell goal = {5, 14};

    std::vector<GridCell> path = findPath(start, goal);

    if (path.empty()) {
        SDL_Log("A*: No valid path found!");
    } else {
        SDL_Log("A*: Path found with %d cells.", (int)path.size());
    }

    // Current waypoint along the AI path
    size_t waypointIndex = path.size() > 1 ? 1 : path.size();

    // AI navigation settings
    bool aiEnabled = true;
    const float aiSpeed = 2.0f;
    bool dynamicObstacleAdded = false;

    bool running = true;
    SDL_Event event;

    Uint64 previousTime = SDL_GetTicks();

    
    while (running) {
        
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }

            if (event.type == SDL_EVENT_KEY_DOWN &&
                event.key.key == SDLK_ESCAPE) {
                running = false;
            }

            
            if (event.type == SDL_EVENT_KEY_DOWN &&
                event.key.key == SDLK_O && !event.key.repeat && 
                !dynamicObstacleAdded) {

                Obstacle newObstacle = {
                    450.0f,
                    425.0f,
                    100.0f,
                    100.0f
                };

                obstacles.push_back(newObstacle);
                dynamicObstacleAdded = true;

                
                b2BodyDef newObstacleDef = b2DefaultBodyDef();

                // Convert the obstacle center from pixels to meters
                newObstacleDef.position = {
                    (newObstacle.x + newObstacle.width / 2.0f) / 100.0f,
                    (newObstacle.y + newObstacle.height / 2.0f) / 100.0f
                };

                // Create the static physics body
                b2BodyId newObstacleBody =
                    b2CreateBody(worldId, &newObstacleDef);

                // Create its collision shape
                b2Polygon newObstacleBox = b2MakeBox(
                    newObstacle.width / 200.0f,
                    newObstacle.height / 200.0f
                );

                b2ShapeDef newObstacleShapeDef = b2DefaultShapeDef();

                b2CreatePolygonShape(
                    newObstacleBody,
                    &newObstacleShapeDef,
                    &newObstacleBox
                );

                SDL_Log("New obstacle added! Total obstacles: %d",
                        static_cast<int>(obstacles.size()));

                // Get the robot's current Box2D position
                b2Vec2 currentPosition = b2Body_GetPosition(robotBody);

                // Convert Box2D meters to grid coordinates
                GridCell currentCell = {
                    static_cast<int>(currentPosition.y * 100.0f) / CELL_SIZE,
                    static_cast<int>(currentPosition.x * 100.0f) / CELL_SIZE
                };

                // Recalculate the path using the updated obstacle vector
                path = findPath(currentCell, goal);

                // Start following the new path
                waypointIndex = 1;
                if (path.empty()) {
                    SDL_Log("A*: No valid path after adding obstacle!");
                    waypointIndex = 0;
                } else {
                    SDL_Log("A*: New path found with %d cells.",
                            (int)path.size());
                }

            }


            if (event.type == SDL_EVENT_KEY_DOWN &&
                !event.key.repeat) {

                // Press M for manual control
                if (event.key.key == SDLK_M) {
                    aiEnabled = false;
                    SDL_Log("Manual mode activated");
                }

                // Press P for AI navigation
                if (event.key.key == SDLK_P) {
                    aiEnabled = true;

                    // Get current robot position
                    b2Vec2 pos = b2Body_GetPosition(robotBody);

                    GridCell currentCell = {
                        std::clamp(static_cast<int>(pos.y * 100.0f) / CELL_SIZE,
                                0, GRID_ROWS - 1),
                        std::clamp(static_cast<int>(pos.x * 100.0f) / CELL_SIZE,
                                0, GRID_COLS - 1)
                    };


                    bool onExistingPath = false;

                    for (size_t i = 0; i < path.size(); ++i) {
                        if (path[i].row == currentCell.row &&
                            path[i].col == currentCell.col) {

                            waypointIndex = i + 1;
                            onExistingPath = true;

                            SDL_Log("AI: Rejoining existing path at cell %d",
                                    static_cast<int>(i));
                            break;
                        }
                    }

                    if (!onExistingPath) {
                        path = findPath(currentCell, goal);
                        waypointIndex = path.size() > 1 ? 1 : path.size();

                        SDL_Log("AI: New path calculated from current position");
                    }

                    SDL_Log("AI mode activated");
                }

                // Press R to reset the robot
                if (event.key.key == SDLK_R) {
                    b2Body_SetTransform(robotBody, {4.0f, 3.0f},
                                        b2MakeRot(0.0f));
                    b2Body_SetLinearVelocity(robotBody, {0.0f, 0.0f});

                    path = findPath(start, goal);
                    waypointIndex = path.size() > 1 ? 1 : path.size();
                    aiEnabled = true;

                    SDL_Log("Robot reset to starting position - AI mode activated");
                }
            }
        }

        // Calculate elapsed time
        Uint64 currentTime = SDL_GetTicks();
        float deltaTime =
            (currentTime - previousTime) / 1000.0f;
        previousTime = currentTime;

        // Limit unusually large physics time steps
        if (deltaTime > 0.05f) {
            deltaTime = 0.05f;
        }

        // SDL3 reads the keyboard
        const bool* keys = SDL_GetKeyboardState(nullptr);

        // Box2D movement velocity (meters per second)
        b2Vec2 velocity = {0.0f, 0.0f};

        if (aiEnabled && waypointIndex < path.size()) {

            // Current robot position in Box2D meters
            b2Vec2 current = b2Body_GetPosition(robotBody);

            // Center of the next A* grid cell
            GridCell next = path[waypointIndex];

            float targetX =
                (next.col * CELL_SIZE + CELL_SIZE / 2.0f) / 100.0f;

            float targetY =
                (next.row * CELL_SIZE + CELL_SIZE / 2.0f) / 100.0f;

            float dx = targetX - current.x;
            float dy = targetY - current.y;

            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 0.05f) {
                // Waypoint reached; move to the next one
                waypointIndex++;
            } else {
                // Move toward the waypoint
                float speed = std::min(
                    aiSpeed,
                    distance / std::max(deltaTime, 0.001f)
                );

                velocity.x = (dx / distance) * speed;
                velocity.y = (dy / distance) * speed;
            }
        }
        else if (!aiEnabled) {

            // Manual movement using WASD or arrow keys
            const float manualSpeed = 2.5f;

            if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) {
                velocity.y = -manualSpeed;
            }

            if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) {
                velocity.y = manualSpeed;
            }

            if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) {
                velocity.x = -manualSpeed;
            }

            if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) {
                velocity.x = manualSpeed;
            }
        }
        // Apply robot movement through Box2D
        b2Body_SetLinearVelocity(robotBody, velocity);

        // Advance the physics simulation
        if (deltaTime > 0.0f) {
            b2World_Step(worldId, deltaTime, 4);
        }

        // Get the robot's position from Box2D
        b2Vec2 position = b2Body_GetPosition(robotBody);

        // Convert meters to pixels
        robotX = position.x * 100.0f - robotSize / 2.0f;
        robotY = position.y * 100.0f - robotSize / 2.0f;
        
        // Step 44.2: Detect when the AI robot gets stuck
        static b2Vec2 previousPosition = position;
        static float stuckTimer = 0.0f;
        static bool stuckWarningShown = false;

        float movementX = position.x - previousPosition.x;
        float movementY = position.y - previousPosition.y;

        float distanceMoved = std::sqrt(
            movementX * movementX +
            movementY * movementY
        );

        // Only check when AI is actively following a path
        if (aiEnabled && waypointIndex < path.size()) {

            if (distanceMoved < 0.001f) {
                stuckTimer += deltaTime;
            }
            else {
                stuckTimer = 0.0f;
                stuckWarningShown = false;
            }

            
            if (stuckTimer >= 2.0f && !stuckWarningShown) {
                SDL_Log("WARNING: Robot appears to be stuck!");

                stuckWarningShown = true;

                // Determine the robot's current grid cell
                GridCell currentCell = {
                    std::clamp(
                        static_cast<int>(position.y * 100.0f) / CELL_SIZE,
                        0, GRID_ROWS - 1
                    ),
                    std::clamp(
                        static_cast<int>(position.x * 100.0f) / CELL_SIZE,
                        0, GRID_COLS - 1
                    )
                };

                // Try to calculate a new route
                
                GridCell avoidCell = path[waypointIndex];

                SDL_Log("A*: Avoiding waypoint row=%d, col=%d",
                        avoidCell.row, avoidCell.col);

                std::vector<GridCell> newPath =
                    findPath(currentCell, goal, avoidCell);


                if (!newPath.empty()) {
                    bool avoidedCellFound = false;

                    for (const GridCell& cell : newPath) {
                        if (cell.row == avoidCell.row &&
                            cell.col == avoidCell.col) {
                            avoidedCellFound = true;
                            break;
                        }
                    }
                    SDL_Log("A*: Recovery path contains avoided cell: %s",
                        avoidedCellFound ? "YES" : "NO");
                    path = newPath;
                    waypointIndex = path.size() > 1 ? 1 : path.size();

                    SDL_Log("A*: Recovery path found with %d cells.",
                            static_cast<int>(path.size()));
                }
                else {
                    SDL_Log("A*: Recovery failed - no valid path.");
                    path.clear();
                    waypointIndex = 0;
                }

                // Reset the timer
                stuckTimer = 0.0f;
            }
        }
        else {
            stuckTimer = 0.0f;
            stuckWarningShown = false;
        }

        previousPosition = position;

        // Check whether the robot overlaps the green goal
        const float goalLeft = 700.0f;
        const float goalTop = 275.0f;
        const float goalRight = 730.0f;
        const float goalBottom = 305.0f;

        bool goalReached =
            robotX < goalRight &&
            robotX + robotSize > goalLeft &&
            robotY < goalBottom &&
            robotY + robotSize > goalTop;

        
        
        // Determine the current game title
        const char* newTitle;

        if (goalReached) {
            newTitle = "AI Robot Arena - Goal Reached!";
        }
        else if (aiEnabled) {
            newTitle = "AI Robot Arena - AI Mode";
        }
        else {
            newTitle = "AI Robot Arena - Manual Mode";
        }

        // Change the title only when necessary
        static std::string currentTitle = "";

        if (currentTitle != newTitle) {
            SDL_SetWindowTitle(window, newTitle);
            currentTitle = newTitle;
        }

        
        // Log success once each time the robot enters the goal
        static bool goalLogged = false;

        if (goalReached && !goalLogged) {
            SDL_Log("SUCCESS: Robot reached the goal!");
            goalLogged = true;
        }
        else if (!goalReached) {
            goalLogged = false;
        }
        
        // SDL3 draws the background
        SDL_SetRenderDrawColor(renderer, 25, 30, 45, 255);
        SDL_RenderClear(renderer);

        // Draw the A* planned path
        SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);

        for (const GridCell& cell : path) {
            SDL_FRect marker = {
                cell.col * CELL_SIZE + 20.0f,
                cell.row * CELL_SIZE + 20.0f,
                10.0f,
                10.0f
            };

            SDL_RenderFillRect(renderer, &marker);
        }
        

        // Draw all obstacles using SDL3
        SDL_SetRenderDrawColor(renderer, 130, 140, 155, 255);

        for (const Obstacle& obstacle : obstacles) {
            SDL_FRect rect = {
                obstacle.x,
                obstacle.y,
                obstacle.width,
                obstacle.height
            };

            SDL_RenderFillRect(renderer, &rect);
        }

        // Draw the AI navigation goal
        SDL_FRect goalRect = {
            700.0f, 275.0f, 30.0f, 30.0f
        };

        SDL_SetRenderDrawColor(renderer, 0, 220, 100, 255);
        SDL_RenderFillRect(renderer, &goalRect);

        // SDL3 draws the robot at its Box2D position
        SDL_FRect robot = {
            robotX, robotY, robotSize, robotSize
        };

        SDL_SetRenderDrawColor(renderer, 0, 180, 255, 255);
        SDL_RenderFillRect(renderer, &robot);

        SDL_RenderPresent(renderer);

        SDL_Delay(16);
    }


    b2DestroyWorld(worldId);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
