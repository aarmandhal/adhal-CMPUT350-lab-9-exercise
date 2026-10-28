#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // Reset camera to last frame's unadjusted center
        camera.setCameraCenter(cameraUnadjustedCenter);

        // Get player's x position as a percentage of window width
        float playerPercentX = camera.worldToWindowPercent(trackedPosWorld).x;

        // Get direction of player's velocity
        float dirSign;
        if (trackedVelocityWorld.x > 0.f) {
            // Moving right, look ahead to the right
            dirSign = 1.f;
        } else if (trackedVelocityWorld.x < 0.f) {
            // Moving left, look ahead to the left
            dirSign = -1.f;
        } else {
            // Stopped, no lookahead
            dirSign = 0.f;
        }

        // Calculate target lookahead offset in world coordinates
        float lookAheadTargetWorld = camera.windowPercentToWorldDist({dirSign * lookAhead, 0.f}).x;
        // Smooth lookahead state using EMA
        lookaheadState = lookaheadState + alpha * (lookAheadTargetWorld - lookaheadState);

        // Calculate new unadjusted x position
        float newUnadjustedX = cameraUnadjustedCenter.x;

        // Determine which zone the player's in and adjust the unadjusted camera center accordingly
        if (deadZoneX.contains(playerPercentX)) {
            // dead zone: no change
        } else if (softZoneX.contains(playerPercentX)) {
            // soft zone: smooth transition to dead zone boundary
            float violatedPercent =
                (playerPercentX < deadZoneX.min) ? deadZoneX.min : deadZoneX.max;
            float targetWorldX = camera.windowPercentToWorld({violatedPercent, 0.f}).x;
            newUnadjustedX =
                cameraUnadjustedCenter.x + alpha * (targetWorldX - cameraUnadjustedCenter.x);
        } else {
            // hard zone: move directly to boundary
            float violatedPercent =
                (playerPercentX < softZoneX.min) ? softZoneX.min : softZoneX.max;
            float targetWorldX = camera.windowPercentToWorld({violatedPercent, 0.f}).x;
            newUnadjustedX = targetWorldX;
        }

        // Update camera center with lookahead state
        camera.setCameraCenter({newUnadjustedX + lookaheadState, cameraUnadjustedCenter.y});
        // Store the unadjusted center for the next frame
        cameraUnadjustedCenter = {newUnadjustedX, cameraUnadjustedCenter.y};
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif  // DYNAMIC_CAMERA_H
