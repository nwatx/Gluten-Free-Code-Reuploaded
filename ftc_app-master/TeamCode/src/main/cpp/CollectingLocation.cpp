//
// Created by peter on 3/8/2019.
//

#include "CollectingLocation.h"
#include "MineralScorer.h"

/**
 * Creates a new collecting location
 *
 */
CollectingLocation::CollectingLocation() {
    score = 0;//initialize our score to 0
}

/**
 * sets where we are collecting the cubes from
 * @param worldCoordinate the cube location in world space
 */
void CollectingLocation::setCollectingLocation(Point2d worldCoordinate) {
    collectingLocation = worldCoordinate;
    collectingLocationScreen = MineralScorer::pointToImage(worldCoordinate);
}

/**
 * Sets our score
 * @param score
 */
void CollectingLocation::setScore(double score) {
    this->score = score;
}

/**
 * Sets the drop zone in world coordinates
 * @param dropZone the point to drop the collector
 */
void CollectingLocation::setDropZone(Point2d dropZone) {
    this->dropZone = dropZone;
    this->dropZoneScreen = MineralScorer::pointToImage(dropZone);
}
