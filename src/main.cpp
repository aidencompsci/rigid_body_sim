#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <utility>
#include <vector>

#if defined(PLATFORM_WEB)
    #include <emscripten/emscripten.h>
#endif

Vector2 Vector2Copy(Vector2 v) {
    return {
        .x = v.x,
        .y = v.y,
    };
}

typedef struct {
    Vector2 pos;
    Vector2 last;
    Vector2 acc;
    Color color;
    float radius;
    int id;
} VerletObject;

void VerletObject_update(VerletObject* self, float dt) {
    auto disp = Vector2Subtract(self->pos, self->last);
    self->last = self->pos;
    self->pos.x += disp.x + self->acc.x * (dt * dt);
    self->pos.y += disp.y + self->acc.y * (dt * dt);
    self->acc.x = 0;
    self->acc.y = 0;
}

void VerletObject_accelerate(VerletObject* self, Vector2 new_acceleration) {
    self->acc = Vector2Add(self->acc, new_acceleration);
}

void VerletObject_velocity_set(VerletObject* self, Vector2 v, float dt) {
    self->last = Vector2Subtract(self->pos, Vector2Scale(v, dt));
}

void VerletObject_velocity_add(VerletObject* self, Vector2 v, float dt) {
    self->last = Vector2Subtract(self->last, Vector2Scale(v, dt));
}

Vector2 VerletObject_velocity_get(VerletObject* self, float dt) {
    return Vector2Divide(
            Vector2Subtract(self->pos, self->last),
            Vector2{dt, dt}
            );
}

const int MAX_CELL_OBJECTS = 1000;
struct Cell {
    std::vector<VerletObject*> objects;
    Cell() {
        objects.reserve(MAX_CELL_OBJECTS);
    }
};

const int GRID_WIDTH = 100;
const int GRID_HEIGHT = 100;
const int CELL_SIZE = 10;
class SpacialHashGrid {
public:
    std::vector<Cell> cells;
    int total = 0;

    SpacialHashGrid() = default;
    SpacialHashGrid(size_t cellCount) : cells(cellCount) {}

    inline int getCellX(const VerletObject& ob) {
        return static_cast<int>(ob.pos.x / CELL_SIZE);
    }

    inline int getCellY(const VerletObject& ob) {
        return static_cast<int>(ob.pos.y / CELL_SIZE);
    }

    inline int getCellIndex(const VerletObject& ob) {
        int x = getCellX(ob);
        int y = getCellY(ob);
        int index = x + (y * GRID_WIDTH);
        return index;
    }

    inline int getCellXY(int x, int y) {
        int index = x + (y * GRID_WIDTH);
        return index;
    }

    bool insert(VerletObject& ob) {
        int index = getCellIndex(ob);
        if (index < 0 || index > GRID_WIDTH*GRID_HEIGHT) {
            return false;
        }

        cells[index].objects.push_back(&ob);
        total++;
        return true;
    }

    void clear() {
        total = 0;
        for (auto& cell : cells) {
            cell.objects.clear();
        }
    }

    bool rebuild(std::vector<VerletObject>& objects) {
        clear();

        for (auto& ob : objects) {
            if (!insert(ob)) {
                return false;
            };
        }

        return true;
    }

    int query(const VerletObject& ob, VerletObject* results[], int capacity) {
        int cx = getCellX(ob);
        int cy = getCellY(ob);
        int index = getCellIndex(ob);
        if (index < 0 || index > GRID_WIDTH*GRID_HEIGHT) {
            return 0;
        }

        int resultCount = 0;

        for (int l = cy - 1; l <= cy + 1; l++) {
            if (l < 0 || l >= GRID_HEIGHT) continue;
            for (int i = cx - 1; i <= cx + 1; i++) {
                if (i < 0 || i >= GRID_WIDTH) continue;
                int index = getCellXY(i, l);
                for (auto ob2 : cells[index].objects) {
                    if (ob.id == ob2->id) continue;
                    results[resultCount] = ob2;
                    resultCount++;
                }
            }
        }

        return resultCount;
    }
};

const Vector2 gravity = Vector2{0, 1000};
typedef struct{
    std::vector<VerletObject> objects;
    SpacialHashGrid grid;
    Vector2 constraint_center;
    float constraint_radius;
    u_int32_t sub_steps;
    bool should_constrain;
} Solver;


void Solver_apply_gravity(Solver* self) {
    for (auto& ob : self->objects) {
        VerletObject_accelerate(&ob, gravity);
    }
}

void Solver_move_objects(Solver* self, float dt) {
    for (auto& ob : self->objects) {
        VerletObject_update(&ob, dt);
    }
}

void Solver_apply_constraint(Solver* self) {
    for (auto& ob : self->objects) {
        const Vector2 v = Vector2Subtract(self->constraint_center, ob.pos);
        const float dist = Vector2Length(v);
        if (dist > (self->constraint_radius - ob.radius)) {
            ob.pos = Vector2Subtract(
                    self->constraint_center,
                    Vector2Scale(
                        Vector2Normalize(v),
                        (self->constraint_radius - ob.radius)
                        )
                    );
        }
    }
}

#define VECTOR2_SUBTRACT(a, b) \
    (Vector2{(a).x - (b).x, (a).y - (b).y})

#define VECTOR2_LENGTH_SQUARED(a) \
    ((a).x*(a).x+(a).y*(a).y)

const float resp_coef = 0.75;
// [[inline]]
// inline __attribute__((always_inline)) void resolve_collision(VerletObject& ob1, VerletObject& ob2) {
void resolve_collision(VerletObject& ob1, VerletObject& ob2) {
    const Vector2 v = VECTOR2_SUBTRACT(ob1.pos, ob2.pos);
    const float dist2 = VECTOR2_LENGTH_SQUARED(v);
    const float min_dist = ob1.radius + ob2.radius;

    if (dist2 < min_dist*min_dist) {
        const float dist = sqrt(dist2);
        const auto norm = Vector2Normalize(v);
        //radius == mass
        const float mr1 = ob1.radius / (ob1.radius + ob2.radius);
        const float mr2 = ob2.radius / (ob1.radius + ob2.radius);
        const float delta = 0.5f * resp_coef * (dist - min_dist);
        ob1.pos = Vector2Subtract(ob1.pos, Vector2Scale(norm, (mr2 * delta)));
        ob2.pos = Vector2Add(ob2.pos, Vector2Scale(norm, (mr1 * delta)));
    }
}


const int RESULTS_CAPACITY = MAX_CELL_OBJECTS * 9;
VerletObject* results[RESULTS_CAPACITY];
void Solver_check_collisions(Solver* self) {
    for (auto& ob1 : self->objects) {
        int count = self->grid.query(ob1, results, RESULTS_CAPACITY);
        for (int i = 0; i < count && i < RESULTS_CAPACITY; i++) {
            auto ob2 = *results[i];
            const Vector2 v = VECTOR2_SUBTRACT(ob1.pos, ob2.pos);
            const float dist2 = VECTOR2_LENGTH_SQUARED(v);
            const float min_dist = ob1.radius + ob2.radius;

            if (dist2 < min_dist*min_dist) {
                const float dist = sqrt(dist2);
                const auto norm = Vector2Normalize(v);
                //radius == mass
                const float mr1 = ob1.radius / (ob1.radius + ob2.radius);
                const float mr2 = ob2.radius / (ob1.radius + ob2.radius);
                const float delta = 0.5f * resp_coef * (dist - min_dist);
                ob1.pos = Vector2Subtract(ob1.pos, Vector2Scale(norm, (mr2 * delta)));
                ob2.pos = Vector2Add(ob2.pos, Vector2Scale(norm, (mr1 * delta)));
            }
        }
    }
}

void Solver_Update(Solver* self, float dt) {
    const float step_dt = dt / static_cast<float>(self->sub_steps);
    for (u_int32_t i{self->sub_steps}; i--;) {
        Solver_apply_gravity(self);
        self->grid.rebuild(self->objects);
        Solver_check_collisions(self);
        if (self->should_constrain) {
            Solver_apply_constraint(self);
        }
        Solver_move_objects(self, step_dt);
    }
}


static Color getRainbow(float time) {
    const float r = sin(time);
    const float g = sin(time + 0.33f * 2.0f * PI);
    const float b = sin(time + 0.66f * 2.0f * PI);
    return ColorFromNormalized(Vector4 {r, g, b, 1.0});
}

typedef struct {
    Vector2 pos;
    Solver world;
    int spawn_count;
} GameState;


const int screenWidth = 1000;
const int screenHeight = 1000;
GameState game_state;

void Init() {
    InitWindow(screenWidth, screenHeight, "raylib + rlImGui + ImGui Example");
    rlImGuiSetup(true);
    SetTargetFPS(144);


    game_state = GameState{
        .pos = Vector2{600, 292},
        .world = Solver{
            .objects = {},
            .grid = SpacialHashGrid(GRID_WIDTH*GRID_HEIGHT),
            .constraint_center = {500, 500},
            .constraint_radius = 450,
            .sub_steps = 8,
            .should_constrain = true,
        },
        .spawn_count = 1,
    };

    game_state.world.objects.reserve(10000);
}

void Update() {
    Solver_Update(&game_state.world, GetFrameTime());
}

int counter = 0;
void GuiDraw() {
    ImGui::Begin("Debug");
    ImGui::SliderFloat("Pos.x", &game_state.pos.x, 0, static_cast<float>(GetScreenWidth()));
    ImGui::SliderFloat("Pos.y", &game_state.pos.y, 0, static_cast<float>(GetScreenHeight()));
    ImGui::DragInt("Spawn Count", &game_state.spawn_count);
    if (ImGui::Button("Spawn Object at position")) { // || ImGui::IsItemActive()) {
    // if (ImGui::Button("Spawn Object at position") || ImGui::IsItemActive()) {
        for (int i = 0; i < game_state.spawn_count; ++i) {
            game_state.world.objects.push_back(VerletObject{
                    .pos = Vector2Copy(game_state.pos),
                    .last = Vector2Copy(game_state.pos),
                    .acc = {0, 0},
                    .color = getRainbow((GetTime()*2)+2),
                    .radius = 10,
                    .id = counter++,
            });
        }
    }
    if (ImGui::Button("Toggle Constraint")) {
        game_state.world.should_constrain = !game_state.world.should_constrain;
    }
    if (ImGui::Button("Clear objects")) {
        game_state.world.objects.clear();
    }
    ImGui::End();
}

int gamecount = 0;
void GameDraw() {
    for (auto& ob : game_state.world.objects) {
        DrawCircleV(ob.pos, ob.radius, ob.color);
    }

    DrawCircleV(game_state.pos, 10, PURPLE);
}

void Draw() {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    GameDraw();

    rlImGuiBegin();
    GuiDraw();
    rlImGuiEnd();

    DrawFPS(12, 12);
    DrawText(("count: " + std::to_string(game_state.world.objects.size())).c_str(), 12, 12+20+4, 20, BLACK);
    DrawText(("frame time: " + std::to_string(GetFrameTime())).c_str(), 12, 12+20+20+4, 20, BLACK);


    EndDrawing();
}

void UpdateDrawFrame() {
    Update();
    Draw();
}

void Cleanup() {
    rlImGuiShutdown();
    CloseWindow();
}


int main() {
    Init();

#if defined (PLATFORM_WEB)
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else 
    while (!WindowShouldClose()) {
        UpdateDrawFrame();
    }
#endif

    Cleanup();

    return 0;
}
