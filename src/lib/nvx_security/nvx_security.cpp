#include "nvx_security.h"

#include <px4_platform_common/atomic.h>

namespace
{
px4::atomic_bool g_service_mode{false};
}

namespace nvx_security
{

Mode mode()
{
	return g_service_mode.load() ? Mode::Service : Mode::Production;
}

bool service_mode()
{
	return g_service_mode.load();
}

void set_service_mode(bool enabled)
{
	g_service_mode.store(enabled);
}

const char *mode_name()
{
	return service_mode() ? "Service" : "Production";
}

} // namespace nvx_security
