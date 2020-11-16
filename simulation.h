#ifndef _SIMULATION__H
#define  _SIMULATION__H

#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>
#include <list>
#include <vector>

#define COLLISION 10
#define OK 0
#define MAX_TRACER_LENGTH 500
#define TIME_BETWEEN_TRACES 20000000

#ifdef DEBUG
#define DEBUG_PRINT printf
#else
#define DEBUG_PRINT //
#endif

enum
{
	SIM_STATE_STOPPED,
	SIM_STATE_RUNNING,
	SIM_STATE_PAUSED
};

class program_state
{
public:
	void SetRunning(uint32_t state);

public:
	uint32_t sim_running; // is the simulation allowed to continue
	uint32_t fps_target; // our target frame rate
	uint64_t total_time; // total running time of the simulation (ns)
	uint64_t trials; 		// how many times have we run the simulation (this session)
	bool quit;				// quit the program
	uint64_t update_rate; // forced rate (ns) for updating the simulation
	bool realtime;			// use the wall clock, or update_rate
	bool ui_visible;		// use graphical mode
	char *trace_filename; // file to use to record sensor trace
};

class CSimulation;
typedef void(*event_cb)(CSimulation *s, uint64_t id, uint64_t timestamp);

struct point
{
	point(double _x, double _y) : x(_x), y(_y) {}
	double x;
	double y;
};

class sim_object
{
public:
	sim_object(double x, double y) :
		m_name(NULL), m_pos_x(x), m_pos_y(y), m_velocity_x(0), m_velocity_y(0), m_acceleration_x(0), m_acceleration_y(0), m_scale(10.0),
		m_tracerLength(0), m_width(1), m_height(1), m_tracer_elapsed_ns(0), m_max_velocity_x(100), m_max_velocity_y(100) { }
	virtual void Update(uint64_t nsec);
	virtual void Draw(SDL_Renderer* renderer);
	void set_name(const char* name) { m_name = strdup(name); }
	void set_width(uint32_t w) { m_width = w; }
	void set_height(uint32_t h) { m_height = h; }
	void set_velocity_x(double v) { m_velocity_x = v; }
	void set_velocity_y(double v) { m_velocity_y = v; }
	void set_acceleration_x(double v) { m_acceleration_x = v; }
	void set_acceleration_y(double v) { m_acceleration_y = v; }
	void set_tracer_length(uint32_t len) { if (len < MAX_TRACER_LENGTH) m_tracerLength = len; }
	void accelerate_to_position(uint64_t dest_x, uint64_t dest_y);
	void accelerate_to_velocity(uint64_t dest_x, uint64_t dest_y);

//////
	const char* name() { return m_name; }
	double x() { return m_pos_x; }
	double y() { return m_pos_y; }
	double width() { return m_width; }
	double height() { return m_height; }
	double velocity_x() { return m_velocity_x; }
	double velocity_y() { return m_velocity_y; }
	double acceleration_x() { return m_acceleration_x; }
	double acceleration_y() { return m_acceleration_y; }

protected:
	char *m_name;
	uint32_t m_color;
	uint32_t m_width;
	uint32_t m_height;
	double m_scale;
	double m_pos_x;
	double m_pos_y;
	double m_velocity_x;
	double m_velocity_y;
	double m_acceleration_x;
	double m_acceleration_y;
	double m_max_velocity_x;
	double m_max_velocity_y;
	double m_dest_pos_x;
	double m_dest_pos_y;

	uint32_t m_tracerLength;
	std::vector<point> m_tracerPts;
	uint64_t m_tracer_elapsed_ns;
};

class sim_event
{
public:
	sim_event(uint64_t timestamp, uint64_t id, event_cb cb) : m_timestamp(timestamp), m_id(id), m_cb(cb){}

public:
	uint64_t m_timestamp;
	uint64_t m_id;
	event_cb m_cb;
};

struct sim_collision
{
	uint64_t timestamp;
	uint64_t x;
	uint64_t y;
	sim_object* a;
	sim_object* b;
	bool draw; // should we draw on the ui where this collision occurred
};

typedef std::list<sim_object*> obj_list;
typedef std::list<sim_event*> event_list;

class CSimulation
{
public:
	CSimulation() : m_state(NULL) {}
	virtual bool Initialize(program_state *state, uint32_t w, uint32_t h);
	virtual uint64_t UpdateSimulation(uint64_t abs_ns, uint64_t elapsed_ns);
	virtual void Draw(SDL_Renderer* r);
	virtual void HandleEvent(SDL_Event *event) = 0;
	virtual void OnCollision(uint64_t abs_ns, sim_object *a, sim_object *b)=0;
	bool who_collided(sim_collision *c, sim_object *a, sim_object *b);

protected:
	bool CheckForCollision(uint64_t abs_ns);

protected:
	program_state *m_state;
	uint64_t m_width;
	uint64_t m_height;
	obj_list sim_objects;
	event_list sim_events; // schedule of things that happen, and when
};


#endif // _SIMULATION__H
