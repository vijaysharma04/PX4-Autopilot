#pragma once

namespace nvx_security
{

enum class Mode {
	Production = 0,
	Service
};

Mode mode();
bool service_mode();
void set_service_mode(bool enabled);
const char *mode_name();

} // namespace nvx_security
