#include "arkheon/character/ICharacterController.h"
#include <cstring>
#include <cmath>

namespace {
constexpr uint8_t HID_W = 26;

struct Vec3 {
    float x, y, z;
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
};

struct Quat {
    float x, y, z, w;
};

Quat normalize(Quat q) {
    float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-6f) return {0.0f, 0.0f, 0.0f, 1.0f};
    return {q.x / len, q.y / len, q.z / len, q.w / len};
}

Vec3 quat_to_angular_error(Quat q_cur, Quat q_ref) {
    float dot = q_cur.x * q_ref.x + q_cur.y * q_ref.y + q_cur.z * q_ref.z + q_cur.w * q_ref.w;
    if (dot < 0.0f) {
        q_ref.x = -q_ref.x; q_ref.y = -q_ref.y; q_ref.z = -q_ref.z; q_ref.w = -q_ref.w;
    }
    return {
        2.0f * (q_ref.x * q_cur.w - q_ref.w * q_cur.x - q_ref.y * q_cur.z + q_ref.z * q_cur.y),
        2.0f * (q_ref.y * q_cur.w - q_ref.w * q_cur.y - q_ref.z * q_cur.x + q_ref.x * q_cur.z),
        2.0f * (q_ref.z * q_cur.w - q_ref.w * q_cur.z - q_ref.x * q_cur.y + q_ref.y * q_cur.x)
    };
}

struct JointPhysicsState {
    Quat  q_cur = {0.0f, 0.0f, 0.0f, 1.0f};
    Vec3  w_cur = {0.0f, 0.0f, 0.0f};
    float Kp = 80.0f;
    float Kd = 8.0f;
    float inertia = 0.05f;
};

struct Controller {
    float             seg_len[10] = {0};
    JointPhysicsState joints[10];
    
    // GOTO Görev ve Navmesh Durumları
    int32_t           last_seq_id = -1;
    arkheon_vec3      path_buffer[32]; // Rotayı tutacağımız dizi
    int32_t           path_len = 0;
    int32_t           path_idx = 0;
    
    int32_t           active_motion = 0;
    float             walk_speed = 1.4f;

    Controller() {
        for (int i = 0; i < 10; ++i) {
            if (i == ARK_JOINT_CALF_L || i == ARK_JOINT_CALF_R ||
                i == ARK_JOINT_LOWERARM_L || i == ARK_JOINT_LOWERARM_R) {
                joints[i].Kp = 120.0f;
                joints[i].Kd = 12.0f;
            } else {
                joints[i].Kp = 70.0f;
                joints[i].Kd = 7.0f;
            }
        }
    }
};
}

extern "C" {

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void) {
    return ARKHEON_CHARACTER_SDK_VERSION;
}

ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void) {
    return "Student Plugin v0.4 (GOTO Navigation) - Kübra Bayrak";
}

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(void* /*h*/, int32_t out[3]) {
    out[0] = 12; // walk_forward
    out[1] = 47; // push_two_handed
    out[2] = 83; // climb_low_step
}

ARKHEON_CHAR_EXPORT void* arkheon_character_create(const float segs[10]) {
    auto* c = new Controller();
    std::memcpy(c->seg_len, segs, sizeof(c->seg_len));
    return c;
}

ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* h) {
    delete static_cast<Controller*>(h);
}

ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
    void* h,
    const arkheon_frame*        frame,
    const arkheon_bone_state    in_bones[66],
    arkheon_bone_override       out_overrides[10],
    arkheon_vec3*               out_root_translation_delta,
    arkheon_quat*               out_root_rotation_delta,
    const arkheon_input_state*  input,
    const arkheon_mission_goal* goal,
    const arkheon_env_api*      env)
{
    auto* c = static_cast<Controller*>(h);
    float dt = static_cast<float>(frame->delta_time_s);

    *out_root_translation_delta = {0.0f, 0.0f, 0.0f};
    *out_root_rotation_delta    = {0.0f, 0.0f, 0.0f, 1.0f};
    
    // Karakterin anlık dünya pozisyonunu 0. kemikten (genelde Root/Pelvis) alıyoruz
    Vec3 current_root_pos = {in_bones[0].world_position.x, in_bones[0].world_position.y, in_bones[0].world_position.z};

    // 1. Yeni Bir Görev Geldiyse Yolu (Path) Hesapla
    if (goal && goal->sequence_id != c->last_seq_id) {
        c->last_seq_id = goal->sequence_id;
        
        if (goal->type == ARK_GOAL_GOTO && env && env->navmesh_query) {
            arkheon_vec3 start_pos = {current_root_pos.x, current_root_pos.y, current_root_pos.z};
            // Ortamdan hedef koordinata kadar olan köşe noktalarını (waypoints) iste
            c->path_len = env->navmesh_query(env->host_ctx, start_pos, goal->target_position, c->path_buffer, 32);
            c->path_idx = 0;
        }
    }

    // 2. Otonom Yürüme ve Yönelme (Navmesh Following)
    bool is_navigating = false;
    if (goal && goal->type == ARK_GOAL_GOTO && c->path_len > 0 && c->path_idx < c->path_len) {
        is_navigating = true;
        
        // Sıradaki hedef noktayı belirle
        Vec3 target = {c->path_buffer[c->path_idx].x, c->path_buffer[c->path_idx].y, c->path_buffer[c->path_idx].z};
        Vec3 diff = target - current_root_pos;
        diff.y = 0; // Sadece X ve Z (yatay) düzlemde mesafeye bak
        
        float dist = std::sqrt(diff.x * diff.x + diff.z * diff.z);
        
        if (dist < 0.3f) { 
            // Hedefe çok yaklaştık, sıradaki noktaya geç
            c->path_idx++;
        }
        
        if (c->path_idx >= c->path_len) {
            // Son noktaya ulaştık. Toplam tolerans payındaysak görevi tamamla[cite: 7]
            Vec3 final_target = {goal->target_position.x, goal->target_position.y, goal->target_position.z};
            Vec3 final_diff = final_target - current_root_pos;
            final_diff.y = 0;
            float final_dist = std::sqrt(final_diff.x * final_diff.x + final_diff.z * final_diff.z);
            
            if (final_dist <= goal->tolerance_m) {
                if (env && env->report_goal_complete) {
                    env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK); //[cite: 7]
                }
                c->path_len = 0; // Hareketi durdur
            }
        } else if (dist > 0.001f) {
            // Hedefe doğru ilerle
            float move_step = c->walk_speed * dt;
            if (move_step > dist) move_step = dist; // Hedefi aşmayı engelle
            
            out_root_translation_delta->x = (diff.x / dist) * move_step;
            out_root_translation_delta->z = (diff.z / dist) * move_step;
            
            // Karakterin gövdesini yürüdüğü yöne döndür (Y ekseninde Yaw açısı hesabı)
            float target_yaw = std::atan2(diff.x, diff.z);
            out_root_rotation_delta->x = 0.0f;
            out_root_rotation_delta->y = std::sin(target_yaw / 2.0f);
            out_root_rotation_delta->z = 0.0f;
            out_root_rotation_delta->w = std::cos(target_yaw / 2.0f);
        }
    }

    // 3. Otonom hareket yoksa Manuel Tuş Kontrolü
    if (!is_navigating && input) {
        if (input->hotkey_motion_a) c->active_motion = 0;
        if (input->hotkey_motion_b) c->active_motion = 1;
        if (input->hotkey_motion_c) c->active_motion = 2;

        if (input->keys[HID_W]) {
            float yaw = input->look_yaw_rad;
            out_root_translation_delta->x = std::sin(yaw) * c->walk_speed * dt;
            out_root_translation_delta->z = std::cos(yaw) * c->walk_speed * dt;
        }
    }

    // 4. Eklemleri Fizik Motoruyla Hareket Ettir
    for (int i = 0; i < 10; ++i) {
        auto& j = c->joints[i];
        Quat q_ref = {0.0f, 0.0f, 0.0f, 1.0f};
        Vec3 w_ref = {0.0f, 0.0f, 0.0f};

        Vec3 err_ang = quat_to_angular_error(j.q_cur, q_ref);
        Vec3 err_vel = w_ref - j.w_cur;
        Vec3 torque = (err_ang * j.Kp) + (err_vel * j.Kd);

        Vec3 alpha = torque * (1.0f / j.inertia);
        j.w_cur = j.w_cur + (alpha * dt);

        j.q_cur.x += 0.5f * ( j.w_cur.x * j.q_cur.w + j.w_cur.y * j.q_cur.z - j.w_cur.z * j.q_cur.y) * dt;
        j.q_cur.y += 0.5f * (-j.w_cur.x * j.q_cur.z + j.w_cur.y * j.q_cur.w + j.w_cur.z * j.q_cur.x) * dt;
        j.q_cur.z += 0.5f * ( j.w_cur.x * j.q_cur.y - j.w_cur.y * j.q_cur.x + j.w_cur.z * j.q_cur.w) * dt;
        j.q_cur.w += 0.5f * (-j.w_cur.x * j.q_cur.x - j.w_cur.y * j.q_cur.y - j.w_cur.z * j.q_cur.z) * dt;
        j.q_cur = normalize(j.q_cur);

        out_overrides[i].apply = 1;
        out_overrides[i].local_rotation = {j.q_cur.x, j.q_cur.y, j.q_cur.z, j.q_cur.w};
    }

    return 0;
}

}