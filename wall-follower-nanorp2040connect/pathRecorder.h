#pragma once
#include "debounce.h"

// Class to encapdulate the recording of a line-follower course
// and facilitate faster replay using known contours
class PathRecorder
{
public:
  typedef enum {
    startMark,
    endMark,
    forward,
    rightTurn,
    rightTurn180,
    rightTurn270,
    leftTurn,
    leftTurn180,
    leftTurn270,
    crossOver
  } SegmentDirection;

  typedef struct  
    {
      int positionLeft;
      int positionRight;
      int distanceLeft;
      int distanceRight;
      SegmentDirection direction;
      int turn;
    } Segment;

private:

  bool pursuitMode;
  
  static const int MAX_SEGEMENTS = 100;
  Segment segments[MAX_SEGEMENTS];
  int totalSegments;
  int currentSegment;

  int lastPositionLeft;
  int lastPositionRight;

  int lastSegmentDeferredPositionLeft;
  int lastSegmentDeferredPositionRight;

  // Edge detection helpers
  Debounce startFinish;
  Debounce radiusMarker;

  void addSegment(SegmentDirection direction);
  void adjustSegmentDistances();

public:
  PathRecorder();

  // Recorder
  void reset(bool pursuitMode);
  void record(int radiusMarkerReading, int startStopMarkerReading, int positionLeft, int positionRight);
  bool detectedEndMarker() const;

  // Playback
  SegmentDirection getFirstSegment();
  SegmentDirection getNextSegment();
  SegmentDirection peakNextSegment();
  int getSegmentDistance();
  int getNextSegmentDistance();
  int getCurrentSegmentDistance();  // during record only
  bool isSegmentEndMarker();
  int currentSegmentNumber() { return currentSegment; }
  static bool isDirectionForward(SegmentDirection direction);

  int countCrossovers() const;

  // Debug
  void printPath();
  static void printDirection(SegmentDirection direction);

};
