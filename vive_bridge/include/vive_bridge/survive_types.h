#include "os_generic.h"
#include <survive_types.h>

// Simple API structures
struct SurviveExternalObject {
	SurvivePose pose;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type {
		SurviveSimpleObject_LIGHTHOUSE,
		SurviveSimpleObject_OBJECT,
		SurviveSimpleObject_EXTERNAL
	} type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;
};

struct SurviveSimpleContext {
	SurviveContext* ctx; 
	
	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t external_object_ct;
	struct SurviveSimpleObject *external_objects;

	size_t object_ct;
	struct SurviveSimpleObject objects[];
};