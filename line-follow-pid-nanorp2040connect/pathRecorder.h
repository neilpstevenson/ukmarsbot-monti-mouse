#include "debounce.h"

// Class to encapdulate the recording of a line-follower course
// and facilitate faster replay using known contours
class PathRecorder
{
private:
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

  static const int MAX_SEGEMENTS = 100;

  typedef struct  
    {
      int positionLeft;
      int positionRight;
      int distanceLeft;
      int distanceRight;
      SegmentDirection direction;
      int turn;
    } Segment;

    Segment segments[MAX_SEGEMENTS];
    int totalSegments;
    int currentSegment;

    int lastPositionLeft;
    int lastPositionRight;
  
    // Edge detection helpers
    Debounce startFinish;
    Debounce radiusMarker;

  void addSegment(SegmentDirection direction);
  void adjustSegmentDistances();
  
public:
  PathRecorder();

  void reset();
  void record(int radiusMarkerReading, int startStopMarkerReading, int positionLeft, int positionRight);
  bool detectedEndMarker() const;

  void printPath();

};
