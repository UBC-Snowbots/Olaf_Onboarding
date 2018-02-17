/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: Tests for OlafNode
 */

#include <OlafNode.h>
#include <gtest/gtest.h>

TEST(MyNode, addExclamationPoint){
    EXPECT_EQ("!", OlafClass::addCharacterToString("", "!"));
    EXPECT_EQ("Hello!", OlafClass::addCharacterToString("Hello", "!"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}