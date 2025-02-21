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
    // Color color;
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

Color colorFromXY(int x, int y, Color a = BLACK, Color b = RED) {
    auto tx = x % 2;
    auto ty = y % 2;
    auto tt = tx + ty;
    auto color = tt == 1 ? a : b;
    return color;
}

Color colorFromXY(float x, float y, Color a = BLACK, Color b = RED) {
    return colorFromXY(static_cast<int>(x), static_cast<int>(y), a, b);
}

constexpr int SCREEN_WIDTH = 1000;
constexpr int SCREEN_HEIGHT = 1000;
constexpr int CELL_SIZE = 25;
constexpr int CELL_COUNT_X = SCREEN_WIDTH / CELL_SIZE;
constexpr int CELL_COUNT_Y = SCREEN_HEIGHT / CELL_SIZE;
constexpr int CELL_COUNT = CELL_COUNT_X * CELL_COUNT_Y;
constexpr int CELL_INIT_COUNT = 1000;
class SpacialHashGrid {
public:
    std::vector<std::vector<VerletObject*>> cells;
    int total = 0;

    SpacialHashGrid() : cells(CELL_COUNT) {
        for (auto& cell : cells) {
            cell.reserve(CELL_INIT_COUNT);
        }
    };

    inline static int getCellX(const VerletObject& ob) {
        return ob.pos.x / CELL_SIZE;
    }

    inline static int getCellY(const VerletObject& ob) {
        return ob.pos.y / CELL_SIZE;
    }

    inline static int getCellIndex(const VerletObject& ob) {
        auto cx = getCellX(ob);
        auto cy = getCellY(ob);
        return getCellIndexXY(cx, cy);
    }

    inline static int getCellIndexXY(int cx, int cy) {
        return cx + (cy * CELL_COUNT_Y);
    }

    bool insert(VerletObject& ob) {
        auto index = getCellIndex(ob);
        if (index < 0 || index >= CELL_COUNT) {
            return false;
        }

        cells[index].push_back(&ob);
        return true;
    }

    void clear() {
        for (auto& cell : cells) {
            cell.clear();
        }
    }

    bool rebuild(std::vector<VerletObject>& objects) {
        clear();

        for (auto& ob : objects) {
            insert(ob);
            // if (!insert(ob)) {
            //     return false;
            // };
        }

        return true;
    }

    int query(const VerletObject& ob, VerletObject* results[], int capacity) {
        const auto obcx = getCellX(ob);
        const auto obcy = getCellY(ob);

        int count = 0;

        for (int cx = obcx - 1; cx <= obcx + 1; cx++) {
            for (int cy = obcy - 1; cy <= obcy + 1; cy++) {
                auto index = getCellIndexXY(cx, cy);
                if (index < 0 || index > CELL_COUNT) continue;
                auto cell = cells[index];
                for (auto& ob2 : cell) {
                    if (count > capacity-1) return count;
                    // if (ob.id != ob2->id) {
                        results[count] = ob2;
                        count++;
                    // }
                }
            }
        }

        return count;
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
        const float dist = sqrt(v.x*v.x + v.y*v.y);
        const auto pen_depth = self->constraint_radius - ob.radius;
        if (dist > pen_depth) {
            const auto n = Vector2{
                .x = v.x / dist,
                .y = v.y / dist,
            };
            const auto correction = Vector2Scale(n, pen_depth);
            ob.pos = Vector2Subtract(self->constraint_center, correction);
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


// const int MAX_CELL_OBJECTS = 2*CELL_SIZE*CELL_SIZE;
// const int RESULTS_CAPACITY = MAX_CELL_OBJECTS * 9;
const int RESULTS_CAPACITY = 1000;
VerletObject* results[RESULTS_CAPACITY];
void Solver_check_collisions(Solver* self) {
    int max_count = 0;
    for (auto& ob1 : self->objects) {
        // self->grid.rebuild(self->objects);
        int count = self->grid.query(ob1, results, RESULTS_CAPACITY);
        if (count > max_count) max_count = count;
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
    std::cout << "max count: " << max_count << "\n";
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
    bool should_draw_grid;
} GameState;


GameState game_state;

void Init() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "raylib + rlImGui + ImGui Example");
    rlImGuiSetup(true);
    SetTargetFPS(144);


    game_state = GameState{
        .pos = Vector2{515, 500},
        .world = Solver{
            .objects = {},
            .grid = SpacialHashGrid(),
            .constraint_center = {500, 500},
            .constraint_radius = 450,
            .sub_steps = 1,
            .should_constrain = true,
        },
        .spawn_count = 1,
    };

    // game_state.world.objects.reserve(10000);
}

int counter = 0;
void addBall(int offx = 0, int offy = 0) {
    for (int x = game_state.pos.x-offx; x <= game_state.pos.x+offx; x++) {
        for (int y = game_state.pos.y-offy; y <= game_state.pos.y+offy; y++) {
            game_state.world.objects.push_back(VerletObject{
                .pos = Vector2{.x=static_cast<float>(x), .y=static_cast<float>(y)},
                .last = Vector2{.x=static_cast<float>(x), .y=static_cast<float>(y)},
                .acc = {0, 0},
                // .color = getRainbow((GetTime() * 2) + 2),
                // .radius = static_cast<float>(GetRandomValue(2, 5)),
                .radius = 10,
                .id = counter++,
            });
        }
    }
}

void Update() {
    Solver_Update(&game_state.world, GetFrameTime());
    // if (IsMouseButtonDown(MouseButton::MOUSE_BUTTON_RIGHT)) {
    //     auto mpos = GetMousePosition();
    //     game_state.pos = mpos;
    //     if (IsMouseButtonDown(MouseButton::MOUSE_BUTTON_LEFT)) {
    //         for (int i = 0; i < game_state.spawn_count; ++i) {
    //             addBall(2, 2);
    //         }
    //     }
    // }
}

void GuiDraw() {
    ImGui::Begin("Debug");
    ImGui::SliderFloat("Pos.x", &game_state.pos.x, 0, static_cast<float>(GetScreenWidth()));
    ImGui::SliderFloat("Pos.y", &game_state.pos.y, 0, static_cast<float>(GetScreenHeight()));
    ImGui::DragInt("Spawn Count", &game_state.spawn_count);
    // if (ImGui::Button("Spawn Object at position")) { // || ImGui::IsItemActive()) {
    if (ImGui::Button("Spawn Object at position") || ImGui::IsItemActive()) {
        for (int i = 0; i < game_state.spawn_count; ++i) {
            addBall();
        }
    }
    if (ImGui::Button("Toggle Constraint")) {
        game_state.world.should_constrain = !game_state.world.should_constrain;
    }
    if (ImGui::Button("Clear objects")) {
        game_state.world.objects.clear();
    }
    if (ImGui::Button("Deccelerate")) {
        for (auto& ob : game_state.world.objects) {
            ob.acc = {0, 0};
            ob.last = ob.pos;
        }
    }
    if (ImGui::Button("Toggle grid draw")) {
        game_state.should_draw_grid = !game_state.should_draw_grid;
    }
    ImGui::End();
}

constexpr int pad = 2;
int gamecount = 0;
void GameDraw() {
    if (game_state.should_draw_grid) {
        for (int x = 0; x < CELL_COUNT_X; x++) {
            for (int y = 0; y < CELL_COUNT_Y; y++) {
                auto color = colorFromXY(x, y, GRAY, PINK);
                DrawRectangle(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE-pad, CELL_SIZE-pad, color);
            }
        }
    }

    for (auto& ob : game_state.world.objects) {
        // auto temp = static_cast<float>(SpacialHashGrid::getCellIndex(ob))/GRID_WIDTH*GRID_HEIGHT;
        // const auto color = ColorFromHSV(temp, 0.8, 1.0);
        const auto cx = static_cast<float>(SpacialHashGrid::getCellX(ob));
        const auto cy = static_cast<float>(SpacialHashGrid::getCellY(ob));
        // const auto x = (ob.pos.x - cx * CELL_SIZE) / static_cast<float>(CELL_SIZE);
        // const auto y = (ob.pos.y - cy * CELL_SIZE) / static_cast<float>(CELL_SIZE);
        const auto color = ColorFromNormalized(Vector4{cx / CELL_COUNT_X, cy / CELL_COUNT_Y, 1.0, 1.0});
        // const auto color = colorFromXY(ob.pos.x / CELL_SIZE, ob.pos.y / CELL_SIZE);
        DrawCircleV(ob.pos, ob.radius, color);
        // std::cout << ob.id << ": " << cx << ", " << cy << " = " << SpacialHashGrid::getCellIndex(ob) << "\n";
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
