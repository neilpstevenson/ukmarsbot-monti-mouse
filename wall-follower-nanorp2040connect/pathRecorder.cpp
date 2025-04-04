#include <Arduino.h>
#include "pathRecorder.h"
#include "defaults.h"

PathRecorder::PathRecorder() :
    pursuitMode(false),
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

void PathRecorder::reset(bool pursuitMode)
{
  totalSegments = 0;
  currentSegment = 0;
  lastPositionLeft = 0;
  lastPositionRight = 0;
  this->pursuitMode = pursuitMode;
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
    //DebugPort.print("Distance adjusted at ");
    //DebugPort.println(getCurrentSegmentDistance());
  }

  if(startFinish.isTriggered(startStopMarkerReading))
  {
      if(pursuitMode)
      {
        if(totalSegments > 1 && 
            segments[totalSegments-1].direction > endMark &&
            segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
        {
          // Radius mark seen, but now realise it is actually a cross-over
          //
          // Convert last mark to a cross-over mark
          segments[totalSegments-1].direction = crossOver;
        }
        else
        {
          // Update last segment
          adjustSegmentDistances();

          // Basically treat as cross-over, as we're not doing anything on start/stop markers
          if(totalSegments)
          {
            addSegment(crossOver);
            DebugPort.print("CROSS Pursuit at ");
          }
          else
          {
            addSegment(crossOver);
            DebugPort.print("CROSS Start Pursuit at ");
          }
          DebugPort.print(positionLeft);
          DebugPort.print(",");
          DebugPort.println(positionRight);
        }
      }
      else
      {
        if(totalSegments > 1 && 
            segments[totalSegments-1].direction > endMark &&
            segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
        {
          // Radius mark seen, but now realise it is actually a cross-over
          //
          // Convert last mark to a cross-over mark
          segments[totalSegments-1].direction = crossOver;
        }
        else
        {
          // Update last segment
          adjustSegmentDistances();
    
          // Add new segment to list
          if(totalSegments)
          {
            addSegment(endMark);
            DebugPort.print("END at ");
          }
          else
          {
            addSegment(startMark);
            DebugPort.print("START at ");
          }
          DebugPort.print(positionLeft);
          DebugPort.print(",");
          DebugPort.println(positionRight);
        }
      }
  }

  // Radius marker check
  if(radiusMarker.isTriggered(radiusMarkerReading))
  {
      if(pursuitMode)
      {
        // Could this be a cross-over?
        if(totalSegments > 1 && 
            (segments[totalSegments-1].direction <= endMark || segments[totalSegments-1].direction == crossOver) &&
            segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
        {
          // Convert last mark to a cross-over mark
          segments[totalSegments-1].direction = crossOver;
          adjustSegmentDistances();
        }
        else
        if(totalSegments == 1 && 
            (segments[totalSegments-1].direction <= endMark || segments[totalSegments-1].direction == crossOver) &&
            segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE > positionLeft)
        {
          // Ignore initial crossover
          adjustSegmentDistances();
        }
        else
        {
          // Update last segment
          adjustSegmentDistances();

          // Add to list
          addSegment(rightTurn);    // Initial guess

          DebugPort.print("Radius at ");
          DebugPort.print(positionLeft);
          DebugPort.print(",");
          DebugPort.println(positionRight);
        }
      }
      else
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

          DebugPort.print("Radius at ");
          DebugPort.print(positionLeft);
          DebugPort.print(",");
          DebugPort.println(positionRight);
        }
      }
  }
}

int PathRecorder::countCrossovers() const
{
  int count = 0;
  for(int segment = 0; segment <= currentSegment; segment++ )
  {
    if(segments[segment].direction == crossOver)
      count++;
  }
  return count;
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
    DebugPort.print("Adjusted dist ");
    printDirection(segments[totalSegments-1].direction);
    DebugPort.print(",");
    DebugPort.print(getCurrentSegmentDistance());
    DebugPort.print("->");
    DebugPort.println(getSegmentDistance());
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
  DebugPort.print("Path Recorded ");
  DebugPort.print(totalSegments);
  DebugPort.println(" segments");
  for(int i = 0; i < totalSegments; i++)   
  {
    DebugPort.print(i);                         DebugPort.print(",");
    printDirection(segments[i].direction);   DebugPort.print(",");
    DebugPort.print(segments[i].positionLeft);  DebugPort.print(",");
    DebugPort.print(segments[i].positionRight); DebugPort.print(",");
    DebugPort.print(segments[i].distanceLeft);  DebugPort.print(",");
    DebugPort.print(segments[i].distanceRight);
    DebugPort.println();
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
      DebugPort.print("Start");   break;
    case endMark:
      DebugPort.print("End");   break;
    case forward:
      DebugPort.print("Fwd");   break;
    case rightTurn:
      DebugPort.print("Right");   break;
    case rightTurn180:
      DebugPort.print("R180");   break;
    case rightTurn270:
      DebugPort.print("R270");   break;
    case leftTurn:
      DebugPort.print("Left");   break;
    case leftTurn180:
      DebugPort.print("L180");   break;
    case leftTurn270:
      DebugPort.print("L270");   break;
    case crossOver:
      DebugPort.print("Cross");   break;
  }
}

bool PathRecorder::detectedEndMarker() const
{
  if(pursuitMode)
  {
    return countCrossovers() >= 4;
  }
  else
  {
    return totalSegments > 1 && 
            segments[totalSegments-1].direction == endMark &&
            segments[totalSegments-1].positionLeft + CROSSOVER_TOLERANCE < lastPositionLeft;
  }
}
