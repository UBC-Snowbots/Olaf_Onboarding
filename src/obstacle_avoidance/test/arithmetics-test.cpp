#include<Arithmetics.h>
#include<gtest/gtest.h>
#include<geometry_msgs/Point32.h>
#include<math.h>

class Points : public ::testing::Test{
	protected:
		virtual void SetUp(){
			point1.x = 0.0;
			point1.y = 0.0;
			point1.z = 0.0;
			point2.x = 2.0;
			point2.y = 0.0;
			point2.z = 0.0;
			point3.x = 0.0;	
			point3.y = 2.0;
			point3.z = 0.0;
		}
		geometry_msgs::Point32 point1;
		geometry_msgs::Point32 point2;
		geometry_msgs::Point32 point3;
};


bool equal(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
	return (point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z);
}

TEST_F(Points, distance){
	ASSERT_EQ(2, Arithmetics::distance(point1, point2));
	ASSERT_EQ(2, Arithmetics::distance(point1, point3));
	ASSERT_FLOAT_EQ(sqrt(8), Arithmetics::distance(point2, point3));
}

TEST_F(Points, compareY){
	EXPECT_FALSE(Arithmetics::compareY(point3, point1));
	EXPECT_TRUE(Arithmetics::compareY(point2, point3));
	EXPECT_FALSE(Arithmetics::compareY(point1, point2));
}

TEST_F(Points, averagePoint){
	geometry_msgs::Point32 point12;
	point12.x = 1;
	point12.y = 0;
	point12.z = 0;
	geometry_msgs::Point32 point23;
	point23.x = 1;
	point23.y = 1;
	point23.z = 0;
	geometry_msgs::Point32 point13;
	point13.x = 0;
	point13.y = 1;
	point13.z = 0;
	
	EXPECT_TRUE(equal(point12, Arithmetics::averagePoint(point1, point2)));
	EXPECT_TRUE(equal(point13, Arithmetics::averagePoint(point1, point3)));
	EXPECT_TRUE(equal(point23, Arithmetics::averagePoint(point2, point3)));
}

TEST_F(Points, linearVelocity){
	geometry_msgs::Vector3 v = Arithmetics::linearVelocity(point2);
	EXPECT_EQ(2.0, v.x);
	EXPECT_EQ(0.0, v.y);
	EXPECT_EQ(0.0, v.z);
}

TEST_F(Points, angularVelocity){
	geometry_msgs::Vector3 v2 = Arithmetics::angularVelocity(point2);
	EXPECT_EQ(0.0, v2.x);
	EXPECT_EQ(0.0, v2.y);
	EXPECT_EQ(0.0, v2.z);
	geometry_msgs::Vector3 v3= Arithmetics::angularVelocity(point3);
	EXPECT_EQ(0.0, v3.x);
	EXPECT_EQ(0.0, v3.y);
	EXPECT_FLOAT_EQ(M_PI_2, v3.z);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
