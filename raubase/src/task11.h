#pragma once

using namespace std;

class Task11
{
public:
  /**
   * destructor */
    ~Task11();
  /** setup and request data */
  void setup();
  /**
   * run this mission */
  void run();
  /**
   * terminate */
  void terminate();

private:
  /**
   * Write a timestamped message to log */
  void toLog(const char * message);
  /// added to log
  int state, oldstate;
  /// private stuff
  // debug print to console
  bool toConsole = true;
  // logfile
  FILE * logfile = nullptr;
  bool setupDone = false;
};

/**
 * Make this visible to the rest of the software */
extern Task11 task11;

