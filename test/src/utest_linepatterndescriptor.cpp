#include <micro/math/numeric.hpp>
#include <micro/test/utils.hpp>
#include <LinePatternCalculator.hpp>

using namespace micro;

namespace {

const LinePatternDescriptor descriptor_LANE_CHANGE = {
    { 2, centimeter_t(16) },
    { 1, centimeter_t(14) },
    { 2, centimeter_t(14) },
    { 1, centimeter_t(12) },
    { 2, centimeter_t(12) },
    { 1, centimeter_t(10) },
    { 2, centimeter_t(10) },
    { 1, centimeter_t(8)  },
    { 2, centimeter_t(8)  }
};

} // namespace

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_0cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(0), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_10cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(10), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_15cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(15), centimeter_t(2));
    ASSERT_EQ(2, validLines.size());
    EXPECT_EQ(1, validLines[0]);
    EXPECT_EQ(2, validLines[1]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_17cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(17), centimeter_t(2));
    ASSERT_EQ(2, validLines.size());
    EXPECT_EQ(1, validLines[0]);
    EXPECT_EQ(2, validLines[1]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_20cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(20), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(1, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_35cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(35), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_101cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(101), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_105cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(105), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_POSITIVE_107cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::POSITIVE, centimeter_t(107), centimeter_t(2));
    ASSERT_EQ(0, validLines.size());
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_0cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(0), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_5cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(5), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_7cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(7), centimeter_t(2));
    ASSERT_EQ(2, validLines.size());
    EXPECT_EQ(1, validLines[0]);
    EXPECT_EQ(2, validLines[1]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_9cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(9), centimeter_t(2));
    ASSERT_EQ(2, validLines.size());
    EXPECT_EQ(1, validLines[0]);
    EXPECT_EQ(2, validLines[1]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_12cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(12), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(1, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_21cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(21), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_101cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(101), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_105cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(105), centimeter_t(2));
    ASSERT_EQ(1, validLines.size());
    EXPECT_EQ(2, validLines[0]);
}

TEST(LinePatternDescriptor, LANE_CHANGE_NEGATIVE_107cm) {
    const LinePatternDescriptor::ValidLinesCount validLines = descriptor_LANE_CHANGE.getValidLines(Sign::NEGATIVE, centimeter_t(107), centimeter_t(2));
    ASSERT_EQ(0, validLines.size());
}
