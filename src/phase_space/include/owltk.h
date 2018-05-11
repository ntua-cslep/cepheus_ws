// owltk.h
// OWL toolkit

#ifndef OWLTK_H
#define OWLTK_H

#include <vector>
#include <string>

#include "owl.h"

typedef std::vector<OWLMarker> Markers;
typedef std::vector<OWLRigid> Rigids;
typedef std::vector<OWLCamera> Cameras;

/*
 * Config: handles all OWL configuration

 * Tracker: handles creating and destroying markers
   id: sent to server when performing tracker actions
   type: tracker types in owl.h
   enable: keeps the enable/disable state of the tracker
   name: (optional) used for display purposes
   features:
   markers: markers associated with the tracker

 * Tracker::Marker: marker info
   markerID: the marker index in owl (bit shifted by tracker index after add)
   ledID: the unique led index in owl

 * Variables
   running: if OWL successfully connects to owld server
   pose: current system pose
   scale: system is in mm, scale 10 would make every mm into a cm
   flags:
   frameRate: frames / second
   frame: last frame number from server
*/
class OWL {
public:

  struct Config {
    
    std::string device;
    float freq;
    int flags;
    int interpolate;
    
    int rpdMode;
    std::string rpdFile;
    
    Config(const std::string &device="", float freq=OWL_MAX_FREQUENCY);
    
    void RPD(int mode=0, const char *file=0);
    
    int Slave();
  };

  class Tracker {
  public:

    struct Marker {
      int markerID, ledID;
      std::vector<float> pos;

      //      Marker(int m=-1, int l=-1);
      Marker(int m, int l);
      Marker(int m, int l, float *p, float s=1);
      Marker(const Marker &m, float s=1);
      void Add(int tracker_id);
    };

    typedef std::pair<size_t,float> Feature;
    typedef std::vector<Feature> Features;
    typedef std::vector<Marker> Markers;

    int id;
    int type;
    bool enable;
    std::string name;

    Features features;
    Markers markers;

    // markers 0 - p_max
    Tracker(int type, int id, int p_max);
    // sparse set
    Tracker(int type, int id, std::vector<int> m);
    // create rigid from array
    Tracker(int type, int id, float rigid[][3], int count, float scale=1);
    Tracker(int type, int id, std::vector<Marker> &rigid, float scale=1);
    //    Tracker(int type, int id, std::vector< std::vector<float> > &rigid, std::vector<int> &ids, float scale=1);
    // create rigid or points from file
    Tracker(int type, int id, const char *file);

    int Create();
    void Destroy();
    void Update(bool flag=1);
    void SetFeature(size_t key, float value, bool reuse=1);

    void AddMarker(int ledId);
    int RemoveMarker(int markerid);

    void Save(std::ostream &out, float scale=1);
    int Save(const char *filename, float scale=1);

    int Load(int type, const char *filename);
  }; // Tracker

  struct Frame {
    size_t state;
    int frame;

    Markers markers;
    Rigids rigids;

    Frame();
    virtual ~Frame();
    virtual void Clear();
    int Update(int new_frame, int flag);
  };

  typedef std::vector<Tracker> Trackers;

  int running;
  float freq;
  int flags;
  float scale;
  float pose[7];

  int frame;
  int error;
  float frameRate;
  int checkCalib;

  Trackers trackers;
  Markers markers;
  Rigids rigids;
  Cameras cameras;

protected:

  Frame *newFrame;
  std::vector<char> buf;
  
public:

  OWL(size_t maxmarkers=128, float scale=1);
  virtual ~OWL();

  void Clear();

  int Init(Config &config, bool enable_stream=1, bool set_freq=1);
  void Finish();

  int InitRPD(Config &config);
  void FinishRPD();

  std::string Error(Config &config, int e, int mode=0);

  int Slave();

  void CreateTrackers();
  void DestroyTrackers(int id=-1);

  int RemoveTracker(int id);
  
  Tracker *FindTracker(int id);

  int SetFrequency(float f);
  void EnableStream(int enable);
  void SetPose(const float *p);

  int GetMarkers(Markers &markers);
  int GetRigids(Rigids &rigids);
  int GetCameras(Cameras &cameras);

  int Handler();

  void Flush();

protected:
  
  virtual int ReadFrame();

}; // OWL

#endif // OWLTK_H
