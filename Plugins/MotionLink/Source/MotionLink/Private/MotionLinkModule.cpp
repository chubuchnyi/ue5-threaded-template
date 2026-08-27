#include "Modules/ModuleManager.h"

// Plugin has no custom module class; the default implementation is enough.
// LoadingPhase PreDefault (see MotionLink.uplugin) so the subsystem is ready
// before default game modules spin up.
IMPLEMENT_MODULE(FDefaultModuleImpl, MotionLink);
