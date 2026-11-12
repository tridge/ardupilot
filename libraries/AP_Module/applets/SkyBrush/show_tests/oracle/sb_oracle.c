// SPDX-License-Identifier: GPL-3.0-or-later
// Writes reference outputs of the C libskybrush for a .skyb file, for the
// Rust port's host tests. Build with build_oracle.sh.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <skybrush/skybrush.h>

static uint32_t lcg_state = 12345;
static uint32_t lcg(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return lcg_state >> 8;
}
static float frand(float lo, float hi)
{
    return lo + (hi - lo) * (lcg() / 16777216.0f);
}

static void pr_vec(FILE* f, sb_vector3_with_yaw_t v)
{
    fprintf(f, " %.9g %.9g %.9g", v.x, v.y, v.z);
}

static void sample(FILE* f, const char* tag, sb_trajectory_player_t* p, float t)
{
    sb_vector3_with_yaw_t pos, vel, acc;
    int r1 = sb_trajectory_player_get_position_at(p, t, &pos);
    int r2 = sb_trajectory_player_get_velocity_at(p, t, &vel);
    int r3 = sb_trajectory_player_get_acceleration_at(p, t, &acc);
    fprintf(f, "%s %.9g %d", tag, t, r1 | r2 | r3);
    pr_vec(f, pos);
    pr_vec(f, vel);
    pr_vec(f, acc);
    fprintf(f, "\n");
}

static void sample_traj(FILE* f, const char* prefix, sb_trajectory_t* traj, int n)
{
    sb_trajectory_player_t p;
    float dur = sb_trajectory_get_total_duration_sec(traj);
    char tag[16];

    if (sb_trajectory_player_init(&p, traj)) {
        return;
    }
    /* sequential pass, including exact segment boundaries */
    snprintf(tag, sizeof(tag), "%sP", prefix);
    for (int i = 0; i < n; i++) {
        sample(f, tag, &p, -0.5f + i * ((dur + 2.5f) / n));
    }
    {
        sb_trajectory_player_t q;
        sb_trajectory_player_init(&q, traj);
        for (int k = 0; k < 60 && sb_trajectory_player_has_more_segments(&q); k++) {
            sb_trajectory_segment_t* s = sb_trajectory_player_get_current_segment(&q);
            sample(f, tag, &p, s->start_time_sec);
            sample(f, tag, &p, s->end_time_sec);
            sb_trajectory_player_build_next_segment(&q);
        }
        sb_trajectory_player_destroy(&q);
    }
    sample(f, tag, &p, INFINITY);
    /* random order pass */
    snprintf(tag, sizeof(tag), "%sR", prefix);
    for (int i = 0; i < n / 3; i++) {
        sample(f, tag, &p, frand(-1.0f, dur + 1.0f));
    }
    sb_trajectory_player_destroy(&p);
}

static void pr_stats(FILE* f, const char* tag, int ret, const sb_trajectory_stats_t* s)
{
    fprintf(f, "%s %d %d %u %.9g %.9g %.9g %.9g %.9g", tag, ret, s->valid_components,
        s->duration_msec, s->duration_sec, s->earliest_above_sec, s->takeoff_time_sec,
        s->landing_time_sec, s->start_to_end_distance_xy);
    pr_vec(f, s->initial_pos);
    pr_vec(f, s->pos_at_landing_time);
    pr_vec(f, s->vel_at_landing_time);
    fprintf(f, "\n");
}

static const float stats_cfg[][4] = {
    /* min_ascent, preferred_descent, takeoff_speed, acceleration */
    { 2500, 2500, 2500, 1000 },
    { 1000, 1000, 2000, 4000 },
    { 5000, 0, 1500, 500 },
    { 0, 3000, 2500, 1000 },
};

static void run_stats(FILE* f, sb_trajectory_t* traj, sb_trajectory_stats_t* out_first)
{
    for (size_t i = 0; i < sizeof(stats_cfg) / sizeof(stats_cfg[0]); i++) {
        sb_trajectory_stats_calculator_t calc;
        sb_trajectory_stats_t stats;
        sb_trajectory_stats_calculator_init(&calc, 1000.0f);
        calc.min_ascent = stats_cfg[i][0];
        calc.preferred_descent = stats_cfg[i][1];
        calc.takeoff_speed = stats_cfg[i][2];
        calc.acceleration = stats_cfg[i][3];
        sb_trajectory_stats_init(&stats);
        int ret = sb_trajectory_stats_calculator_run(&calc, traj, &stats);
        pr_stats(f, "S", ret, &stats);
        if (i == 0) {
            *out_first = stats;
        }
    }
}

static void run_landing(FILE* f, uint8_t* block, size_t len, const sb_trajectory_stats_t* stats0)
{
    static const float offsets[][4] = {
        /* dx, dy, landing velocity, terminal velocity */
        { 300, -200, 1000, 1000 },
        { 0, 0, 500, 0 },
        { -450, 120, 2000, 300 },
    };
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        sb_trajectory_t traj;
        sb_trajectory_stats_t stats = *stats0;
        sb_vector3_t end;
        uint8_t* copy = malloc(len);
        memcpy(copy, block, len);
        sb_trajectory_init(&traj);
        sb_trajectory_update_from_buffer(&traj, copy, len);
        end.x = stats.initial_pos.x + offsets[i][0];
        end.y = stats.initial_pos.y + offsets[i][1];
        end.z = stats.initial_pos.z + 1250;
        int ret = sb_trajectory_replace_end_to_land_at_with_terminal_velocity(
            &traj, &stats, end, offsets[i][2], offsets[i][3]);
        fprintf(f, "L %d %zu ", ret, sb_buffer_size(&traj.buffer));
        for (size_t k = 0; k < sb_buffer_size(&traj.buffer); k++) {
            fprintf(f, "%02x", SB_BUFFER(traj.buffer)[k]);
        }
        fprintf(f, "\n");
        pr_stats(f, "LS", ret, &stats);
        if (ret == 0 && i == 0) {
            sample_traj(f, "L", &traj, 150);
        }
        SB_DECREF_STATIC(&traj);
        free(copy);
    }
}

static void run_cut(FILE* f, uint8_t* block, size_t len)
{
    for (int i = 0; i < 6; i++) {
        sb_trajectory_t traj;
        uint8_t* copy = malloc(len);
        memcpy(copy, block, len);
        sb_trajectory_init(&traj);
        sb_trajectory_update_from_buffer(&traj, copy, len);
        sb_buffer_ensure_owned(&traj.buffer);
        float dur = sb_trajectory_get_total_duration_sec(&traj);
        float t = i == 0 ? 0.0f : (i == 5 ? dur + 1.0f : frand(0.0f, dur));
        int ret = sb_trajectory_cut_at(&traj, t);
        fprintf(f, "C %.9g %d ", t, ret);
        for (size_t k = 0; k < sb_buffer_size(&traj.buffer); k++) {
            fprintf(f, "%02x", SB_BUFFER(traj.buffer)[k]);
        }
        fprintf(f, "\n");
        SB_DECREF_STATIC(&traj);
        free(copy);
    }
}

static void run_lights(FILE* f, uint8_t* buf, size_t len, float dur)
{
    sb_light_program_t prog;
    sb_light_player_t player;
    if (sb_light_program_init(&prog)) {
        return;
    }
    int ret = sb_light_program_update_from_binary_file_in_memory(&prog, buf, len);
    fprintf(f, "LIGHT %d\n", ret);
    if (ret == 0 && sb_light_player_init(&player, &prog) == 0) {
        unsigned long end = (unsigned long)((dur + 5) * 1000);
        for (unsigned long i = 0; i < 400; i++) {
            unsigned long t = i * (end / 400);
            sb_rgb_color_t c = sb_light_player_get_color_at(&player, t);
            fprintf(f, "K %lu %d %d %d\n", t, c.red, c.green, c.blue);
        }
        for (int i = 0; i < 100; i++) {
            unsigned long t = lcg() % (end + 1);
            sb_rgb_color_t c = sb_light_player_get_color_at(&player, t);
            fprintf(f, "KR %lu %d %d %d\n", t, c.red, c.green, c.blue);
        }
        sb_light_player_destroy(&player);
    }
    SB_DECREF_STATIC(&prog);
}

static void run_touches(FILE* f)
{
    for (int i = 0; i < 3000; i++) {
        sb_poly_t poly;
        float xs[4];
        float value;
        int n = 1 + (i % 4);
        float result = -1;
        for (int k = 0; k < 4; k++) {
            xs[k] = frand(-5000, 5000);
        }
        if (i % 5 == 0) {
            /* cubic Bezier crossing / touching a level, like takeoff/landing */
            float a = frand(-3000, 3000), b = frand(-3000, 8000);
            float c = frand(-3000, 8000), d = frand(-3000, 3000);
            sb_poly_make_cubic_bezier(&poly, 1, a, b, c, d);
            value = frand(-3000, 3000);
        } else {
            sb_poly_make(&poly, xs, n);
            value = frand(-5000, 5000);
        }
        int hit = sb_poly_touches(&poly, value, &result);
        fprintf(f, "T %d", poly.num_coeffs);
        for (int k = 0; k < poly.num_coeffs; k++) {
            fprintf(f, " %.9g", poly.coeffs[k]);
        }
        fprintf(f, " %.9g %d %.9g\n", value, hit, hit ? result : -1.0f);
    }
}

int main(int argc, char** argv)
{
    if (argc == 2 && !strcmp(argv[1], "--touches")) {
        run_touches(stdout);
        return 0;
    }
    if (argc != 2) {
        fprintf(stderr, "usage: %s file.skyb | --touches\n", argv[0]);
        return 1;
    }
    FILE* in = fopen(argv[1], "rb");
    if (!in) {
        return 1;
    }
    static uint8_t buf[1 << 20];
    size_t len = fread(buf, 1, sizeof(buf), in);
    fclose(in);

    FILE* f = stdout;
    sb_binary_file_parser_t parser;
    int ret = sb_binary_file_parser_init_from_buffer(&parser, buf, len);
    fprintf(f, "H %d\n", ret);
    if (ret) {
        return 0;
    }
    while (sb_binary_file_is_current_block_valid(&parser)) {
        sb_binary_block_t b = sb_binary_file_get_current_block(&parser);
        fprintf(f, "B %d %u %ld\n", b.type, b.length, (long)b.start_of_body);
        if (sb_binary_file_seek_to_next_block(&parser)) {
            break;
        }
    }

    sb_trajectory_t traj;
    sb_trajectory_init(&traj);
    ret = sb_trajectory_update_from_binary_file_in_memory(&traj, buf, len);
    fprintf(f, "TR %d %u %d\n", ret, sb_trajectory_get_total_duration_msec(&traj),
        sb_trajectory_is_empty(&traj));
    float dur = 0;
    if (ret == 0) {
        sb_trajectory_stats_t stats0;
        dur = sb_trajectory_get_total_duration_sec(&traj);
        sample_traj(f, "", &traj, 300);
        run_stats(f, &traj, &stats0);
        run_landing(f, SB_BUFFER(traj.buffer), sb_buffer_size(&traj.buffer), &stats0);
        run_cut(f, SB_BUFFER(traj.buffer), sb_buffer_size(&traj.buffer));
    }
    SB_DECREF_STATIC(&traj);
    run_lights(f, buf, len, dur);
    sb_binary_file_parser_destroy(&parser);
    return 0;
}
