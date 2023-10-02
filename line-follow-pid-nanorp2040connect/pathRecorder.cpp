#include <Arduino.h>
#include "pathRecorder.h"
#include "defaults.h"

PathRecorder::PathRecorder() :
    totalSegments(0),
    currentSegment(0),
    lastPositionLeft(0),
    lastPositionRight(0),
    #if SENSOR_POLAIRTY_TRUE
    // New sensor board
    startFinish(markerLowThreshold, markerHighThreshold, true),
    radiusMarker(markerLowThreshold, markerHighThreshold, true)
    #else
    // UKMARSBOT board
    startFinish(markerLowThreshold, markerHighThreshold, false),
    radiusMarker(markerLowThreshold, markerHighThreshold, false)
    #endif
{

}

void PathRecorder::reset()
{
  totalSegments = 0;
  currentSegment = 0;
  lastPositionLeft = 0;
  lastPositionRight = 0;
}

void PathRecorder::record(int radiusMarkerReading, int startStopMarkerReading, int positionLeft, int positionRight)
{
  // Record max/min to determine what happened after the marker
  lastPositionLeft = positionLeft;
  lastPositionRight = positionRight;

   if(startFinish.isTriggered(startStopMarkerReading))
  {
      // Could this be a cross-over?
      if(totalSegments > 1 && 
          segments[totalSegments-1].direction > endMark &&
          segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
      {
        // Convert last mark to a cross-over mark
        segments[totalSegments-1].direction = crossOver;
      }
      else
      {
        // Update last segment
        adjustSegmentDistances();
  
        // Add new segment to list
        addSegment(totalSegments ? endMark : startMark);
 
        Serial.print("Start/stop at ");
        Serial.println(positionLeft);
      }
  }
  // Radius marker check
  if(radiusMarker.isTriggered(radiusMarkerReading))
  {
      // Could this be a cross-over?
      if(totalSegments > 1 && 
          segments[totalSegments-1].direction <= endMark &&
          segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
      {
        // Convert last mark to a cross-over mark
        segments[totalSegments-1].direction = crossOver;
      }
      else
      {
        // Update last segment
        adjustSegmentDistances();

        // Add to list
        addSegment(rightTurn);    // Initial guess

        Serial.print("Radius at ");
        Serial.println(positionLeft);
      }
  }
}

void PathRecorder::addSegment(SegmentDirection direction)
{
  if(totalSegments < MAX_SEGEMENTS-1)
  {
    segments[totalSegments].positionLeft = lastPositionLeft;
    segments[totalSegments].positionRight = lastPositionRight;
    segments[totalSegments].distanceLeft = 0;
    segments[totalSegments].distanceRight = 0;
    segments[totalSegments].direction = direction;
    ++totalSegments;
  }
}

void PathRecorder::adjustSegmentDistances()
{
  // Adjust the last segment to record how far it moved and which direction
  if(totalSegments)
  {
    int distanceLeft = lastPositionLeft - segments[totalSegments-1].positionLeft;
    int distanceRight = lastPositionRight - segments[totalSegments-1].positionRight;
    segments[totalSegments-1].distanceLeft = distanceLeft;
    segments[totalSegments-1].distanceRight = distanceRight;
    if(segments[totalSegments-1].direction > endMark && segments[totalSegments-1].direction != crossOver)
    {
      if(abs(distanceLeft - distanceRight) < STRAIGHT_LINE_TOLERANCE)
        segments[totalSegments-1].direction = forward;
      else
        segments[totalSegments-1].direction = (distanceLeft > distanceRight) ? rightTurn : leftTurn;
    }
  }
}

void PathRecorder::printPath()
{
  Serial.print("Path Recorded ");
  Serial.print(totalSegments);
  Serial.println(" segments");
  for(int i = 0; i < totalSegments; i++)   
  {
    Serial.print(segments[i].positionLeft);   Serial.print(",");
    Serial.print(segments[i].positionRight);   Serial.print(",");
    Serial.print(segments[i].distanceLeft);   Serial.print(",");
    Serial.print(segments[i].distanceRight);   Serial.print(",");
    switch(segments[i].direction)
    {
      case startMark:
        Serial.print("Start");   break;
      case endMark:
        Serial.print("End");   break;
      case forward:
        Serial.print("Fwd");   break;
      case rightTurn:
        Serial.print("Rght");   break;
      case rightTurn180:
        Serial.print("R180");   break;
      case rightTurn270:
        Serial.print("R270");   break;
      case leftTurn:
        Serial.print("Left");   break;
      case leftTurn180:
        Serial.print("L180");   break;
      case leftTurn270:
        Serial.print("L270");   break;
      case crossOver:
        Serial.print("Cros");   break;
    }
    Serial.println();
  }

}
bool PathRecorder::detectedEndMarker() const
{
  return totalSegments > 1 && 
          segments[totalSegments-1].direction == endMark &&
          segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE < lastPositionLeft;
}


