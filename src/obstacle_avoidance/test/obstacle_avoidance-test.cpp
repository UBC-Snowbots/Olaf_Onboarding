/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: Tests for MyNode
 */

#include <MyNode.h>
#include <gtest/gtest.h>

TEST(MyNode, firstTest) {
	AvoidObstacleClass aoc;
	EXPECT_EQ(0, aoc.getAngVelMsg());
	EXPECT_EQ(LINEAR_VEL, aoc.getLinearVelMsg());
	aoc.setLinearVelMsg(12);
	aoc.setAngVelMsg(5);
	EXPECT_EQ(12, aoc.getLinearVelMsg());
	EXPECT_EQ(5, aoc.getAngVelMsg());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}