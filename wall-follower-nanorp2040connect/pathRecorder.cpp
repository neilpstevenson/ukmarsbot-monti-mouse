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
    startFinish(markerLowThreshold, markerLowThreshold + 20, true),
    radiusMarker(markerLowThreshold, markerLowThreshold + 20, true)
    #else
    // UKMARSBOT board
    startFinish(markerLowThreshold, markerLowThreshold + 20, false),
    radiusMarker(markerLowThreshold, markerLowThreshold  + 20, false)
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

  if(getCurrentSegmentDistance() < SEGMENT_START_DEFERRED_DISTANCE)
  {
    // Adjust after we've got a bit into the segment (straight or bend)
    lastSegmentDeferredPositionLeft = positionLeft;
    lastSegmentDeferredPositionRight = positionRight;
    //Serial.print("Distance adjusted at ");
    //Serial.println(getCurrentSegmentDistance());
  }

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
        Serial.print(positionLeft);
        Serial.print(",");
        Serial.println(positionRight);
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
        Serial.print(positionLeft);
        Serial.print(",");
        Serial.println(positionRight);
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
    currentSegment = totalSegments;
    lastSegmentDeferredPositionLeft = lastPositionLeft;
    lastSegmentDeferredPositionRight = lastPositionRight;

    ++totalSegments;
  }
}

void PathRecorder::adjustSegmentDistances()
{
  // Adjust the last segment to record how far it moved and which direction
  if(totalSegments)
  {
    int distanceLeft = lastPositionLeft - lastSegmentDeferredPositionLeft;
    int distanceRight = lastPositionRight - lastSegmentDeferredPositionRight;
    segments[totalSegments-1].distanceLeft = distanceLeft;
    segments[totalSegments-1].distanceRight = distanceRight;
    if(segments[totalSegments-1].direction > endMark && segments[totalSegments-1].direction != crossOver)
    {
      //if(abs(distanceLeft - distanceRight) < STRAIGHT_LINE_TOLERANCE)
      if(abs((distanceLeft + distanceRight)/(distanceLeft - distanceRight)) > STRAIGHT_LINE_TOLERANCE)
        segments[totalSegments-1].direction = forward;
      else
        segments[totalSegments-1].direction = (distanceLeft > distanceRight) ? rightTurn : leftTurn;
    }
    Serial.print("Adjusted dist ");
    printDirection(segments[totalSegments-1].direction);
    Serial.print(",");
    Serial.print(getCurrentSegmentDistance());
    Serial.print("->");
    Serial.println(getSegmentDistance());
  }
}

PathRecorder::SegmentDirection PathRecorder::getFirstSegment()
{
  currentSegment = 0;
  if(!totalSegments)
    return endMark;
  else
    return segments[0].direction;
}

PathRecorder::SegmentDirection PathRecorder::getNextSegment()
{
  if(currentSegment >= totalSegments-1)
    return endMark;

  return segments[++currentSegment].direction;
}

PathRecorder::SegmentDirection PathRecorder::peakNextSegment()
{
  if(currentSegment >= totalSegments-1)
    return endMark;

  return segments[currentSegment+1].direction;
}

int PathRecorder::getSegmentDistance()
{
  if(currentSegment >= totalSegments)
    return 0;

  // Return the biggest of left/right
  return segments[currentSegment].distanceLeft > segments[currentSegment].distanceRight ? segments[currentSegment].distanceLeft : segments[currentSegment].distanceRight;
}

int PathRecorder::getNextSegmentDistance()
{
  if(currentSegment >= totalSegments-1)
    return 0;

  // Return the biggest of left/right
  return segments[currentSegment+1].distanceLeft > segments[currentSegment+1].distanceRight ? segments[currentSegment+1].distanceLeft : segments[currentSegment+1].distanceRight;
}

int PathRecorder::getCurrentSegmentDistance()
{
  if(currentSegment >= totalSegments)
    return 0;

  // Return the biggest of left/right
  return lastPositionLeft - segments[currentSegment].positionLeft > lastPositionRight - segments[currentSegment].positionRight ? 
        lastPositionLeft - segments[currentSegment].positionLeft : 
        lastPositionRight - segments[currentSegment].positionRight;
}

bool PathRecorder::isSegmentEndMarker()
{
   if(currentSegment >= totalSegments-1)
    return true;

  return segments[currentSegment].direction == endMark;
}

void PathRecorder::printPath()
{
  Serial.print("Path Recorded ");
  Serial.print(totalSegments);
  Serial.println(" segments");
  for(int i = 0; i < totalSegments; i++)   
  {
    Serial.print(i);                         Serial.print(",");
    printDirection(segments[i].direction);   Serial.print(",");
    Serial.print(segments[i].positionLeft);  Serial.print(",");
    Serial.print(segments[i].positionRight); Serial.print(",");
    Serial.print(segments[i].distanceLeft);  Serial.print(",");
    Serial.print(segments[i].distanceRight);
    Serial.println();
  }
}

bool PathRecorder::isDirectionForward(PathRecorder::SegmentDirection direction)
{
  return direction == SegmentDirection::startMark ||
         direction == SegmentDirection::endMark ||
         direction == SegmentDirection::forward || 
         direction == SegmentDirection::crossOver;
}

void PathRecorder::printDirection(PathRecorder::SegmentDirection direction)
{
  switch(direction)
  {
    case startMark:
      Serial.print("Start");   break;
    case endMark:
      Serial.print("End");   break;
    case forward:
      Serial.print("Fwd");   break;
    case rightTurn:
      Serial.print("Right");   break;
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
      Serial.print("Cross");   break;
  }
}

bool PathRecorder::detectedEndMarker() const
{
  return totalSegments > 1 && 
          segments[totalSegments-1].direction == endMark &&
          segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE < lastPositionLeft;
}
