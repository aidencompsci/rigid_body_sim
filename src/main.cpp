#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "imgui.h"
#include <cmath>
#include <string>
#include <sys/types.h>
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
    float radius;
    Color color;
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

const Vector2 gravity = Vector2{0, 1000};
typedef struct{
    std::vector<VerletObject> objects;
    Vector2 constraint_center;
    float constraint_radius;
    u_int32_t sub_steps;
} Solver;


void Solver_apply_gravity(Solver* self, float dt) {
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


typedef struct {
    std::vector<std::vector<std::vector<VerletObject>>> objects;
} Grid;



const float resp_coef = 0.75;


[[inline]]
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

void Solver_check_collisions(Solver* self, float dt) {
    auto& objects = self->objects;
    const u_int32_t count = objects.size();

    for (u_int32_t i{0}; i < count; ++i) {
        auto& ob1 = objects[i];
        for (u_int32_t l{i+1}; l < count; ++l) {
            auto& ob2 = objects[l];
            resolve_collision(ob1, ob2);
        }
    }
}

void Solver_Update(Solver* self, float dt) {
    const float step_dt = dt / static_cast<float>(self->sub_steps);
    for (u_int32_t i{self->sub_steps}; i--;) {
        Solver_apply_gravity(self, step_dt);
        Solver_check_collisions(self, step_dt);
        Solver_apply_constraint(self);
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
    int spawn_count;
    Solver world;
} GameState;


const int screenWidth = 1000;
const int screenHeight = 1000;
GameState game_state;

void Init() {
    InitWindow(screenWidth, screenHeight, "raylib + rlImGui + ImGui Example");
    rlImGuiSetup(true);
    SetTargetFPS(144);


    game_state = GameState{
        .pos = Vector2{900, 292},
        .spawn_count = 1,
        .world = Solver{
            .objects = std::vector<VerletObject>(),
            .constraint_center = {500, 500},
            .constraint_radius = 450,
            .sub_steps = 8,
        },
    };
}

void Update() {
    Solver_Update(&game_state.world, GetFrameTime());
}

void GuiDraw() {
    ImGui::Begin("Debug");
    ImGui::SliderFloat("Pos.x", &game_state.pos.x, 0, static_cast<float>(GetScreenWidth()));
    ImGui::SliderFloat("Pos.y", &game_state.pos.y, 0, static_cast<float>(GetScreenHeight()));
    ImGui::DragInt("Spawn Count", &game_state.spawn_count);
    if(ImGui::Button("Spawn Object at position") || ImGui::IsItemActive()) {
        for (int i = 0; i < game_state.spawn_count; ++i) {
            game_state.world.objects.push_back(VerletObject{
                    .pos = Vector2Copy(game_state.pos),
                    .last = Vector2Copy(game_state.pos),
                    .acc = {0, 0},
                    .radius = 10,
                    .color = getRainbow((GetTime()*2)+2),
                    });
        }
    }
    ImGui::End();
}

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
