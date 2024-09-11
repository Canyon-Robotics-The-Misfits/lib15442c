#pragma once

#include <functional>
#include "lib15442c/math/pose.hpp"

using EndCondition = std::function<bool (lib15442c::Pose)>;

/**
 * @brief End the motion if the robot is near a certain point
 * 
 * @param position The position to check against
 * @param threshold How close the robot needs to be
 * @return EndCondition A callback to check for the end condition
 */
EndCondition position_near(lib15442c::Vec position, double threshold);
/**
 * @brief End the motion of the robot is close to facing a certain angle
 * 
 * @param heading The angle to check against
 * @param threshold How close the angle needs to be
 * @return EndCondition A callback to check for the end condition
 */
EndCondition heading_near(lib15442c::Angle heading, lib15442c::Angle threshold);

/**
 * @brief End the motion if either of the end conditions are true
 * 
 * @param a The first end condition
 * @param b The second end condition
 * @return EndCondition A callback to check if either end condition is true
 */
EndCondition either(EndCondition a, EndCondition b);
/**
 * @brief End the motion if both of the end conditions are true
 * 
 * @param a The first end condition
 * @param b The second end condition
 * @return EndCondition A callback to check if both end conditions are true
 */
EndCondition both(EndCondition a, EndCondition b);

/**
 * @brief No custom end condition
 * 
 * @return Always returns false
 */
bool default_end_condition(lib15442c::Pose);