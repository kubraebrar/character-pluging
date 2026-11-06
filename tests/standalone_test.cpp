#include "arkheon/character/ICharacterController.h"
#include <cstdio>
#include <cmath>
#include <chrono>

static int32_t mock_raycast(void*, arkheon_vec3, arkheon_vec3, float, arkheon_vec3* h, arkheon_vec3* n, int32_t* id) {
    *h = {0,0,0}; *n = {0,1,0}; *id = -1; return 0;
}
static int32_t mock_aabb(void*, int32_t, arkheon_vec3* mn, arkheon_vec3* mx) {
    *mn = {-1,-1,-1}; *mx = {1,1,1}; return 1;
}
static int32_t mock_find(void*, const char*) { return 0; }
static int32_t mock_nav(void*, arkheon_vec3, arkheon_vec3 to, arkheon_vec3* path, int32_t) {
    path[0] = to; return 1;
}
static void mock_done(void*, int32_t seq, int32_t res) {
    std::printf("Goal %d completed with result %d\n", seq, res);
}
static arkheon_vec3 mock_grav(void*) { return {0, -9.81f, 0}; }

int main() {
    std::printf("SDK Version = 0x%08x\n", arkheon_character_sdk_version());

    float segs[10] = {0.0f, 0.0f, 0.296595f, 0.296595f, 0.0f, 0.0f, 0.406626f, 0.406626f, 0.433194f, 0.433194f};
    void* h = arkheon_character_create(segs);
    if (!h) { std::printf("FAIL: create failed\n"); return 1; }

    arkheon_bone_state bones[66] = {};
    for (auto& b : bones) b.local_rotation = {0,0,0,1};

    arkheon_input_state input = {};
    arkheon_env_api env = { nullptr, mock_raycast, mock_aabb, mock_find, mock_nav, mock_done, mock_grav };
    arkheon_frame frame = {};
    frame.delta_time_s = 0.02;

    auto t_start = std::chrono::high_resolution_clock::now();

    for (uint64_t i = 0; i < 1000; ++i) {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;
        arkheon_bone_override out[10] = {};
        arkheon_vec3 dt = {0,0,0};
        arkheon_quat dr = {0,0,0,1};

        int rc = arkheon_character_tick(h, &frame, bones, out, &dt, &dr, &input, nullptr, &env);
        if (rc != 0) { std::printf("FAIL: tick %llu rc=%d\n", i, rc); return 1; }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::printf("Average tick duration: %.4f ms\n", total_ms / 1000.0);

    arkheon_character_destroy(h);
    std::printf("PASS: 1000 ticks completed successfully!\n");
    return 0;
}