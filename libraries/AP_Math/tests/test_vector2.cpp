#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

TEST(Vector2Test, IsEqual)
{
    Vector2l v_int1(1, 1);
    Vector2l v_int2(1, 0);
    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(1.0f, 0.0f);

    EXPECT_FALSE(v_int1 == v_int2);
    EXPECT_TRUE(v_int1 == v_int1);
    EXPECT_FALSE(v_float1 == v_float2);
    EXPECT_TRUE(v_float1 == v_float1);
}

TEST(Vector2Test, angle)
{
    EXPECT_FLOAT_EQ(M_PI/2, Vector2f(0, 1).angle());
    EXPECT_FLOAT_EQ(M_PI/4, Vector2f(1, 1).angle());
    EXPECT_FLOAT_EQ(0.0f, Vector2f(1, 0).angle());
    EXPECT_FLOAT_EQ(M_PI*5/4, Vector2f(-1, -1).angle());
    EXPECT_FLOAT_EQ(M_PI*5/4, Vector2f(-5, -5).angle());
}

AP_GTEST_MAIN()
