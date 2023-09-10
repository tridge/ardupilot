#include "math_test.h"


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

class SE23TestParam {
public:
    SE23 se23_1; // First SE23 object
    SE23 se23_2; // Second SE23 object
    SE23 expected; // Expected result of multiplication
};

class SE23Test : public ::testing::TestWithParam<SE23TestParam> {};

static SE23TestParam se23_multiplication_params[] = {
    {
        .se23_1 = SE23(Matrix3f({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}), Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f), 0.5f),
        .se23_2 = SE23(Matrix3f({0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(2.0f, 2.0f, 2.0f), 0.5f),
        .expected = SE23(Matrix3f({0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}), Vector3f(4.0f, 5.5f, 7.0f), Vector3f(6.0f, 7.0f, 8.0f), 0.0f)
    },
    
};


TEST_P(SE23Test, Multiplication)
{
    auto param = GetParam();
    SE23 result = param.se23_1 * param.se23_2;
    
    // Test if equal
    EXPECT_FLOAT_EQ(result.rot().a.x, param.expected.rot().a.x);
    EXPECT_FLOAT_EQ(result.rot().a.y, param.expected.rot().a.y);
    EXPECT_FLOAT_EQ(result.rot().a.z, param.expected.rot().a.z);
    EXPECT_FLOAT_EQ(result.rot().b.x, param.expected.rot().b.x);
    EXPECT_FLOAT_EQ(result.rot().b.y, param.expected.rot().b.y);
    EXPECT_FLOAT_EQ(result.rot().b.z, param.expected.rot().b.z);
    EXPECT_FLOAT_EQ(result.rot().c.x, param.expected.rot().c.x);
    EXPECT_FLOAT_EQ(result.rot().c.y, param.expected.rot().c.y);
    EXPECT_FLOAT_EQ(result.rot().c.z, param.expected.rot().c.z);

    EXPECT_FLOAT_EQ(result.x().x, param.expected.x().x);
    EXPECT_FLOAT_EQ(result.x().y, param.expected.x().y);
    EXPECT_FLOAT_EQ(result.x().z, param.expected.x().z);

    EXPECT_FLOAT_EQ(result.w().x, param.expected.w().x);
    EXPECT_FLOAT_EQ(result.w().y, param.expected.w().y);
    EXPECT_FLOAT_EQ(result.w().z, param.expected.w().z);

    EXPECT_FLOAT_EQ(result.alpha(), param.expected.alpha());
}

INSTANTIATE_TEST_CASE_P(SE23MultiplicationTests,
                        SE23Test,
                        ::testing::ValuesIn(se23_multiplication_params));

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
